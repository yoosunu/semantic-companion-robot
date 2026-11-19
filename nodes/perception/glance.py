"""glance — "안경 쓰기" 의 판단 (순수 계산, 11-18). inspect 노드가 씀, tests/glance_sim.py 로 시험.

  흐름 (11-13 설계, 11-18 Jasper 확정): 바닥 감지 후보가 반복되면 → 고개를 그쪽으로 → YOLOE 잠깐 → 고개 돌아옴. 걸으면서도 (힐끗).
  여기 있는 것
    1. 좌표 바꾸기: 로봇 좌표 (base_footprint: x 앞, y 왼) ↔ 세상 좌표 (odom). 로봇 위치 (X, Y, θ)
         세상 x = X + x·cosθ − y·sinθ,  세상 y = Y + x·sinθ + y·cosθ
       → 로봇이 움직여도 같은 물건은 세상 좌표에서 같은 자리 (11-18 Jasper: "로봇 좌표는 움직이고 odom 은 그대로")
    2. Candidates: 세상 좌표에서 같은 자리 (거리 비례 반경, 최소 30 cm) need (2) 번 → "진짜 후보". 보여야 하는데 안 보이면 miss.
       이미 본 곳 (켜져 있는 동안) 은 다시 안 봄 = 쿨다운 대신 "새로운 것만" (11-18 v2). 로봇에서 max_dist (1.2 m) 넘으면 안 봄
    3. head_for: 바닥 점이 화면 가운데 오게 하는 고개 명령 (팬, 틸트) — ground.py 의 카메라 위치·틸트 0점 그대로
"""
import math

import ground


def to_world(pose, x, y):
    X, Y, th = pose
    c, s = math.cos(th), math.sin(th)
    return X + x * c - y * s, Y + x * s + y * c


def to_robot(pose, wx, wy):
    X, Y, th = pose
    c, s = math.cos(th), math.sin(th)
    dx, dy = wx - X, wy - Y
    return dx * c + dy * s, -dx * s + dy * c


def pick_sharp(shs, n=2, ratio=0.6, extra=3):
    """같은 고개 자세에서 받은 프레임 선명도들 (시간 순) → 쓸 프레임 번호 n 개 (시간 순) 또는 None (더 기다림).
       기준 = **이 장면에서** 가장 선명한 것의 ratio 이상. 11-19: '평소 선명도' (다른 장면) 와 비교하니 흰 벽 쪽은
       원래 밋밋해서 전부 흐림 판정 → 4 초 포기 7 번. 판단 전에 n+1 장은 받음 (둘 다 흔들린 장면이면 셋째가 드러냄).
       n + extra 장 받았는데도 모자라면 가장 선명한 n 장"""
    if len(shs) < n + 1:
        return None
    top = max(shs)
    ok = [i for i, s in enumerate(shs) if s >= ratio * top]
    if len(ok) < n and len(shs) < n + extra:
        return None
    pool = ok if len(ok) >= n else range(len(shs))
    return sorted(sorted(pool, key=lambda i: -shs[i])[:n])


FLOOR_TYPES = ("doormat", "rug", "sock", "towel", "clothes", "plastic bag", "shoe", "slipper")   # = nodes/navigation/hazards.py


def focus_verdict(votes, views, full, hazard, on=0.5, need=2, floor_types=FLOOR_TYPES):
    """따라보기 (11-20 Jasper: 확신 못 한 건 고개로 따라가며 하나만 계속 봄, 확신되면 다음) — 끝낼지 판단.
       votes: {이름: [점수, ...]} 이번 물건을 본 횟수만큼 (십자 물건), views: 본 횟수, full: 통째로 본 적 있음,
       hazard: floor_hazard 표시 상태 ({"ground": True/None, "flagged": bool}) 또는 None (표시 없음)
       → (끝?, 결과 이름 또는 "?", 이유)"""
    good = {n: [c for c in v if c >= on] for n, v in votes.items()}
    good = {n: v for n, v in good.items() if v}
    if sum(len(v) for v in good.values()) < need:                   # 11-20: 이름이 갈려도 (선반·수납함·빨래통) 0.5 넘은 게 두 번이면 확신
        if views >= need and not votes:
            return True, "-", "빈 바닥 (두 번 봐도 아무것도 없음)"
        if views >= 3 and not good:
            return True, "?", f"{views} 번 봐도 {on} 넘는 이름 없음"  # 11-20: shelf 0.23 하나로 17 s 를 다 씀
        return False, "?", "이름 아직"
    name = max(good, key=lambda n: sum(good[n]) / len(good[n]))
    split = f" (이름 갈림: {', '.join(sorted(good))})" if len(good) > 1 else ""
    if name in floor_types and not full:
        return False, name, "통째로 아직 못 봄"
    # 11-20: 바닥 확인 (2 단계) 은 관찰 모드로 접음 → 끝 조건에서 뺌 (가만히 있으면 원래 안 나와 17 s 기다림). 결과만 덧붙임
    extra = "" if name not in floor_types or hazard is None else (
        ", 떠 있음? (관찰)" if hazard.get("flagged") else ", 바닥 확인" if hazard.get("ground") else "")
    return True, name, "이름 확신" + split + extra


def head_for(x, y, tilt_min=-30.0, tilt_max=30.0):
    """로봇 좌표 바닥 점 (m) → (팬°, 틸트°) 명령. 팬 + = 오른쪽 (/pantilt/cmd)"""
    dx = x - ground.PAN_X
    pan = -math.degrees(math.atan2(y - ground.PAN_Y, dx))
    d = math.hypot(dx, y - ground.PAN_Y)
    look = -math.degrees(math.atan2(ground.TILT_H, max(d - ground.LENS_FWD, 0.05)))   # 실제 시선 (아래 −)
    tilt = max(tilt_min, min(tilt_max, look - ground.TILT_OFFSET))
    return pan, tilt


class Candidates:
    """세상 좌표 후보 추적 (11-18 v2: 쓰레기통을 5 번 다시 본 문제 → Jasper "추적을 제대로 쓰면 안 두리번" — 쿨다운 대신 새로운 것만)
       - 같은 자리 반경 = max(match, rel × 로봇에서 거리) (가까이 30 cm, 2 m 면 50 cm): 바닥에 납작하지 않은 물건은
         고개 각도마다 "바닥 닿은 점" 이 20~30 cm 흔들림 (11-18 사진 3·5·8·10·15·16 = 같은 쓰레기통)
       - 사라짐: **보여야 하는데 (visible(x, y)) 안 잡히면** miss. 고개를 돌려 화면 밖이면 안 셈. miss 2 번 연속이면 지움
       - 가장자리에 걸친 덩어리 (edge) 도 후보 — 힐끗해서 가운데에서 다시 본 위치로 기억을 고쳐 씀 (refine)
       - 본 곳 기억: 켜져 있는 동안 계속 (11-18 Jasper: "그냥 안 보게". 옮겨진 물건은 나중에 지도 이름표 층에서), 반경 같은 규칙"""

    def __init__(self, match=0.30, rel=0.25, need=2, max_miss=2, max_age=10.0, memory_s=float("inf"), max_dist=1.2):
        self.match, self.rel, self.need, self.max_miss, self.max_age = match, rel, need, max_miss, max_age
        self.memory_s, self.max_dist = memory_s, max_dist
        self.tracks = []                     # {"x","y","hits","miss","last","area","edge"} 세상 좌표
        self.seen = []                       # (x, y, t, 이름, 점수)
        self.fails = []                      # [x, y, 실패 수] 힐끗 갔는데 영상 못 얻은 자리 (11-19)

    def radius(self, pose, wx, wy):
        x, y = to_robot(pose, wx, wy)
        return max(self.match, self.rel * math.hypot(x, y))

    def remember(self, wx, wy, now, name="", conf=0.0, pose=None):
        self.seen.append((wx, wy, now, name, conf))
        r = self.radius(pose, wx, wy) if pose else self.match
        self.tracks = [t for t in self.tracks if math.hypot(t["x"] - wx, t["y"] - wy) >= r]

    def fail(self, wx, wy, now, pose=None, limit=2):
        """힐끗 실패 (고개는 갔는데 영상을 못 얻고 포기) — 같은 자리 limit 번째면 '?' 로 기억하고 True.
           11-19 실물: 흰 벽 쪽 후보를 4 초 포기 → 1 초 뒤 또 요청, 7 번 반복. Jasper: 2 번 봤는데 모르면 모르는 게 맞음"""
        r = self.radius(pose, wx, wy) if pose else self.match
        f = next((f for f in self.fails if math.hypot(f[0] - wx, f[1] - wy) < r), None)
        if f is None:
            f = [wx, wy, 0]
            self.fails.append(f)
        f[2] += 1
        if f[2] < limit:
            return False
        self.fails.remove(f)
        self.remember(wx, wy, now, "?", 0.0, pose)
        return True

    def known(self, wx, wy, now, pose=None):
        self.seen = [s for s in self.seen if now - s[2] < self.memory_s]
        r = self.radius(pose, wx, wy) if pose else self.match
        return next((s for s in self.seen if math.hypot(wx - s[0], wy - s[1]) < r), None)

    def step(self, now, pose, blobs, visible=None):
        """blobs: [(로봇 x, 로봇 y, 넓이px, edge)] 덩어리마다 가장 가까운 바닥 점. visible(x, y) → 지금 화면에 보여야 하나 (로봇 좌표)
           → 진짜 후보 하나 (가까운 것) 또는 None. 후보 = {"wx","wy","x","y","dist","hits","area","edge"}"""
        matched = set()
        for b in blobs:
            x, y, area = b[0], b[1], b[2]
            edge = bool(b[3]) if len(b) > 3 else False
            wx, wy = to_world(pose, x, y)
            r = max(self.match, self.rel * math.hypot(x, y))
            free = [t for t in self.tracks if id(t) not in matched]
            t = min(free, key=lambda k: math.hypot(k["x"] - wx, k["y"] - wy), default=None)
            if t and math.hypot(t["x"] - wx, t["y"] - wy) < r:
                a = 0.3 if edge else 0.5                         # 가장자리 값은 덜 믿음
                t["x"], t["y"] = (1 - a) * t["x"] + a * wx, (1 - a) * t["y"] + a * wy
                t["hits"] += 1
                t["miss"], t["last"], t["area"] = 0, now, area
                t["edge"] = t["edge"] and edge
            else:
                t = {"x": wx, "y": wy, "hits": 1, "miss": 0, "last": now, "area": area, "edge": edge}
                self.tracks.append(t)
            matched.add(id(t))
        for t in self.tracks:
            if id(t) in matched:
                continue
            x, y = to_robot(pose, t["x"], t["y"])
            if visible is None or visible(x, y):                  # 보여야 하는데 없음 → miss (화면 밖이면 안 셈)
                t["miss"] += 1
        self.tracks = [t for t in self.tracks if t["miss"] < self.max_miss and now - t["last"] <= self.max_age]
        best = None
        for t in self.tracks:
            if t["hits"] < self.need or self.known(t["x"], t["y"], now, pose):
                continue
            x, y = to_robot(pose, t["x"], t["y"])
            d = math.hypot(x, y)
            if d <= self.max_dist and (best is None or d < best["dist"]):
                best = {"wx": t["x"], "wy": t["y"], "x": x, "y": y, "dist": d, "hits": t["hits"], "area": t["area"], "edge": t["edge"]}
        return best
