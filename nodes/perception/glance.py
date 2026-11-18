"""glance — "안경 쓰기" 의 판단 (순수 계산, 11-18). inspect 노드가 씀, tests/glance_sim.py 로 시험.

  흐름 (11-13 설계, 11-18 Jasper 확정): 바닥 감지 후보가 반복되면 → 고개를 그쪽으로 → YOLOE 잠깐 → 고개 돌아옴. 걸으면서도 (힐끗).
  여기 있는 것
    1. 좌표 바꾸기: 로봇 좌표 (base_footprint: x 앞, y 왼) ↔ 세상 좌표 (odom). 로봇 위치 (X, Y, θ)
         세상 x = X + x·cosθ − y·sinθ,  세상 y = Y + x·sinθ + y·cosθ
       → 로봇이 움직여도 같은 물건은 세상 좌표에서 같은 자리 (11-18 Jasper: "로봇 좌표는 움직이고 odom 은 그대로")
    2. Candidates: 세상 좌표에서 match (15 cm) 안에 window (3 s) 동안 need (2) 번 보이면 "진짜 후보" (그림자·깜빡임 거르기)
       이미 본 곳 (memory_s 5 분, 같은 15 cm) 은 다시 안 봄. 로봇에서 max_dist (1.2 m) 넘으면 안 봄
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


def head_for(x, y, tilt_min=-30.0, tilt_max=30.0):
    """로봇 좌표 바닥 점 (m) → (팬°, 틸트°) 명령. 팬 + = 오른쪽 (/pantilt/cmd)"""
    dx = x - ground.PAN_X
    pan = -math.degrees(math.atan2(y - ground.PAN_Y, dx))
    d = math.hypot(dx, y - ground.PAN_Y)
    look = -math.degrees(math.atan2(ground.TILT_H, max(d - ground.LENS_FWD, 0.05)))   # 실제 시선 (아래 −)
    tilt = max(tilt_min, min(tilt_max, look - ground.TILT_OFFSET))
    return pan, tilt


class Candidates:
    def __init__(self, match=0.15, need=2, window=3.0, memory_s=300.0, max_dist=1.2):
        self.match, self.need, self.window, self.memory_s, self.max_dist = match, need, window, memory_s, max_dist
        self.tracks = []                     # {"x","y","hits":[t…],"area"} 세상 좌표
        self.seen = []                       # (x, y, t, 이름, 점수) 이미 본 곳

    def remember(self, wx, wy, now, name="", conf=0.0):
        self.seen.append((wx, wy, now, name, conf))

    def known(self, wx, wy, now):
        self.seen = [s for s in self.seen if now - s[2] < self.memory_s]
        return any(math.hypot(wx - s[0], wy - s[1]) < self.match for s in self.seen)

    def step(self, now, pose, blobs):
        """blobs: [(로봇 x, 로봇 y, 넓이px)] 덩어리마다 가장 가까운 바닥 점 → 진짜 후보 하나 (가까운 것) 또는 None
           후보 = {"wx","wy","x","y","dist","hits","area"} (x, y = 지금 로봇 좌표)"""
        for x, y, area in blobs:
            wx, wy = to_world(pose, x, y)
            t = min(self.tracks, key=lambda k: math.hypot(k["x"] - wx, k["y"] - wy), default=None)
            if t and math.hypot(t["x"] - wx, t["y"] - wy) < self.match:
                n = len(t["hits"])
                t["x"], t["y"] = (t["x"] * n + wx) / (n + 1), (t["y"] * n + wy) / (n + 1)
                t["hits"].append(now)
                t["area"] = area
            else:
                self.tracks.append({"x": wx, "y": wy, "hits": [now], "area": area})
        for t in self.tracks:
            t["hits"] = [h for h in t["hits"] if now - h <= self.window]
        self.tracks = [t for t in self.tracks if t["hits"]]
        best = None
        for t in self.tracks:
            if len(t["hits"]) < self.need or self.known(t["x"], t["y"], now):
                continue
            x, y = to_robot(pose, t["x"], t["y"])
            d = math.hypot(x, y)
            if d <= self.max_dist and (best is None or d < best["dist"]):
                best = {"wx": t["x"], "wy": t["y"], "x": x, "y": y, "dist": d, "hits": len(t["hits"]), "area": t["area"]}
        return best
