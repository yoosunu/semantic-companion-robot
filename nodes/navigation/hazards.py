"""hazards — 센서 사각의 바닥 위험물 판단 (11-19). 순수 계산 (ROS 없음) → tests/hazard_sim.py 로 시험.

  규칙 (11-19 Jasper):
    - 센서 (LiDAR, 나중에 ToF) 가 볼 수 있는 건 센서가 피한다. 비전은 이름표만.
    - 센서가 못 보는 것 (발닦개·양말처럼 LiDAR 17.5 cm 아래, 납작함) 은 비전만 볼 수 있다 → 비전이 costmap 에 찍는다.
    - 기본은 "안 밟기". 밀고 갈지는 나중 (길이 전혀 없을 때만).
  판단:
    ① 이름이 바닥형 (floor_types) — 임시 안전장치. 낮게 떠 있는 물체는 발자국이 멀리 엉뚱하게 찍힘 (11-19 그림)
    ③ (2단계, 11-20) 떠 있나: 표시 근처 바닥 감지 덩어리 (가장 가까운 점) 를 로봇이 다가가며 모음 → floating().
       바닥에 닿은 것 = 같은 자리. 떠 있는 것 (아래 끝 h) = 찍히는 점 P = R + k(X−R), k = H/(H−h) > 1 →
       로봇이 Δ 다가가면 P 가 (k−1)Δ 만큼 로봇 쪽으로 끌려옴 (h 5 cm·Δ 40 cm → 11 cm). 끌려오면 표시를 뺌
    ② 그 모양 안 (+pad) 에 LiDAR 점이 scan_max 개 이하 → 센서가 못 봄 → 표시. 많으면 센서가 봄 → 이름표만
  모양: 납작한 것 (flat) = 마스크 윤곽 전체를 바닥에 내린 것의 볼록 껍질 (바닥에 깔려 있으니 윤곽 전체가 바닥 위)
        나머지 바닥형 (신발류) = 아래 발자국의 볼록 껍질 + min_w 두께
  같은 물건 = 10 cm 키운 모양끼리 닿거나 겹침 (11-20: 같은 발닦개를 화면 양 끝에서 잘려 본 두 조각, 중심 36 cm 떨어져 따로 찍힘).
  통째 (full) vs 잘림 (화면 끝에 닿은 마스크 = 일부만 보임, 11-20 삼각형·조각):
    통째가 오면 → 그걸로 바꿈 (이미 통째면 더 가까이서 본 것. 11-19: 1.2 m 에서 57×38 → 69×59, 가까울수록 정확)
    잘림끼리 → 합침 (볼록 껍질)
    통째가 있는데 잘림이 오면 → 모양은 그대로 (확인만)
  지우기: clear() (수동, /semantic/clear) 만. 자동 지우기는 다음 단계
"""
import math

import numpy as np

FLAT = ("doormat", "rug", "sock", "towel", "clothes", "plastic bag")   # rug: 11-20 YOLOE 가 발닦개를 rug 로 더 자주 부름 (0.67~0.81). Jasper: 둘 다 피함
LOW = ("shoe", "slipper")


def hull(pts):
    """볼록 껍질 (monotone chain). pts N×2 → M×2 (반시계)"""
    p = sorted(set((round(float(x), 4), round(float(y), 4)) for x, y in pts))
    if len(p) < 3:
        return np.array(p, float).reshape(-1, 2)

    def half(seq):
        out = []
        for q in seq:
            while len(out) >= 2 and (out[-1][0] - out[-2][0]) * (q[1] - out[-2][1]) - (out[-1][1] - out[-2][1]) * (q[0] - out[-2][0]) <= 0:
                out.pop()
            out.append(q)
        return out
    lo, up = half(p), half(reversed(p))
    return np.array(lo[:-1] + up[:-1], float)


def thicken(poly, w):
    """점·선처럼 얇은 모양 → 둘레에 w/2 씩 점을 더해 볼록 껍질 (신발 발자국 한 줄 → 띠)"""
    if len(poly) == 0:
        return poly
    a = np.linspace(0, 2 * math.pi, 8, endpoint=False)
    ring = np.column_stack([np.cos(a), np.sin(a)]) * (w / 2)
    return hull(np.vstack([p + ring for p in poly]))


def inside(poly, pts):
    """점들이 볼록 다각형 (반시계) 안인가 → bool 배열"""
    pts = np.asarray(pts, float).reshape(-1, 2)
    if len(poly) < 3 or not len(pts):
        return np.zeros(len(pts), bool)
    ok = np.ones(len(pts), bool)
    for i in range(len(poly)):
        a, b = poly[i], poly[(i + 1) % len(poly)]
        ok &= (b[0] - a[0]) * (pts[:, 1] - a[1]) - (b[1] - a[1]) * (pts[:, 0] - a[0]) >= 0
    return ok


def grow(poly, pad):
    """볼록 다각형을 중심에서 pad m 만큼 키움 (근사: 꼭짓점을 중심 반대로 pad 이동)"""
    if len(poly) < 3 or pad <= 0:
        return poly
    c = poly.mean(0)
    v = poly - c
    n = np.maximum(np.hypot(v[:, 0], v[:, 1]), 1e-6)[:, None]
    return poly + v / n * pad


def area(poly):
    if len(poly) < 3:
        return 0.0
    x, y = poly[:, 0], poly[:, 1]
    return 0.5 * abs(float(np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1))))


def floating(obs, min_move=0.25, thr=0.06, rel=0.12, min_obs=3):
    """obs: [(rx, ry, px, py)] 같은 좌표계 — 로봇 위치, 그때 바닥 감지가 본 덩어리의 가장 가까운 점.
       → (판정, 끌려온 m, 로봇 이동 m). 판정 None = 근거 부족 (로봇이 min_move 만큼 안 다가감, 제자리 회전은 정보 없음),
         True = 떠 있음 (로봇 쪽으로 max(thr, rel×이동) 넘게 끌려옴), False = 바닥.
       기준: 바닥 물체 잡음 ~5 cm (1 m 에서 1 px ≈ 1.3 cm + 마스크·odom) → 6 cm. 12 % = h ≈ 3 cm 이하는 사실상 바닥으로 침"""
    if len(obs) < min_obs:                               # 11-20: 2 개로 판정하면 한 번 튄 값에 흔들림
        return None, 0.0, 0.0
    o = np.asarray(obs, float)
    d = np.hypot(o[:, 2] - o[:, 0], o[:, 3] - o[:, 1])
    i, j = int(np.argmin(d)), int(np.argmax(d))          # 가까이서 본 것, 멀리서 본 것
    move = o[i, :2] - o[j, :2]                           # 그 사이 로봇이 움직인 것 (물체 쪽으로)
    mv = float(np.hypot(*move))
    if mv < min_move:
        return None, 0.0, mv
    pull = -float((o[i, 2:] - o[j, 2:]) @ (move / mv))   # + = 점이 로봇 쪽 (움직임 반대) 으로 끌려옴
    return pull > max(thr, rel * mv), pull, mv


def shape_for(name, outline, footprint, flat=FLAT, min_w=0.06):
    """이름 + (odom 이든 map 이든 같은 좌표의) 윤곽·발자국 → 볼록 다각형 또는 None"""
    if name in flat and len(outline) >= 3:
        h = hull(outline)
        return h if area(h) > 0.002 else thicken(h, min_w)
    if len(footprint):
        return thicken(hull(footprint) if len(footprint) >= 3 else np.asarray(footprint, float), min_w)
    return None


class HazardMap:
    def __init__(self, floor_types=FLAT + LOW, flat=FLAT, on=0.5, scan_max=2, pad=0.03, merge=0.30, max_area=1.5):
        self.types, self.flat = set(floor_types), set(flat)
        self.on, self.scan_max, self.pad, self.merge, self.max_area = on, scan_max, pad, merge, max_area
        self.items = []                       # {"name","conf","poly","t","n"}  poly = map 좌표 볼록 다각형

    def judge(self, name, conf, poly, scan_pts):
        """→ (결과, 이유). 결과: "mark" / "skip". scan_pts = 같은 좌표의 LiDAR 점 N×2"""
        if name not in self.types:
            return "skip", "바닥형 아님"
        if conf < self.on:
            return "skip", f"점수 {conf:.2f} < {self.on}"
        if poly is None or len(poly) < 3:
            return "skip", "모양 없음"
        a = area(poly)
        if a > self.max_area:
            return "skip", f"넓이 {a:.2f} m² 너무 큼 (떠 있는 것의 엉뚱한 발자국일 수 있음)"
        n = int(inside(grow(poly, self.pad), scan_pts).sum())
        if n > self.scan_max:
            return "skip", f"LiDAR 점 {n} 개 = 센서가 봄 → 이름표만"
        return "mark", f"LiDAR 점 {n} 개 = 센서 사각"

    def same(self, a, b, touch, pad=0.10):
        """같은 물건? 중심 merge 안, 또는 (touch 일 때) 각자 pad 키운 모양이 닿거나 겹침.
           touch = 둘 중 하나라도 잘린 조각일 때만 (통째끼리 닿음으로 합치면 옆에 놓인 신발이 발닦개를 덮어씀 — hazard_sim 7)"""
        if math.hypot(*(a.mean(0) - b.mean(0))) < self.merge:
            return True
        if not touch:
            return False
        ga, gb = grow(a, pad), grow(b, pad)
        return bool(inside(ga, gb).any() or inside(gb, ga).any())

    def add(self, name, conf, poly, now, dist=9.9, full=True):
        """표시 더하기 → "new" / "replaced" / "joined" / "kept". dist = 본 거리, full = 통째로 봄 (화면 끝에 안 잘림)"""
        for it in self.items:
            if not self.same(it["poly"], poly, touch=not (full and it["full"])):
                continue
            if full and (not it["full"] or dist < it["dist"]):
                it["poly"], it["dist"], it["full"], how = poly, dist, True, "replaced"
            elif not full and not it["full"]:
                it["poly"], it["dist"], how = hull(np.vstack([it["poly"], poly])), min(it["dist"], dist), "joined"
            else:
                how = "kept"
            if conf >= it["conf"]:
                it["name"], it["conf"] = name, conf
            it["t"], it["n"] = now, it["n"] + 1
            return how
        self.items.append({"name": name, "conf": conf, "poly": poly, "t": now, "n": 1, "dist": dist, "full": full,
                           "obs": [], "ground": None})
        return "new"

    def observe(self, rx, ry, px, py, near=0.20, keep=40):
        """바닥 감지 덩어리 하나 (로봇 위치 + 가장 가까운 점, 표시와 같은 좌표) → 근처 표시에 모으고 떠 있나 판정.
           → [(결과, 표시)] 결과: "floating" (표시 뺌) / "ground" (바닥 확인, 처음 한 번)"""
        out = []
        for it in list(self.items):
            if not inside(grow(it["poly"], near), [(px, py)])[0]:
                continue
            it["obs"] = (it["obs"] + [(rx, ry, px, py)])[-keep:]
            v, pull, mv = floating(it["obs"])
            it["pull"], it["move"] = round(pull, 3), round(mv, 3)
            if v is True:
                self.items.remove(it)
                out.append(("floating", it))
            elif v is False and it["ground"] is None:
                it["ground"] = True
                out.append(("ground", it))
        return out

    def clear(self):
        n = len(self.items)
        self.items = []
        return n

    def cloud_points(self, step=0.03):
        """costmap 에 넣을 점들 (map 좌표): 다각형 안을 step 격자로 채움 + 둘레"""
        out = []
        for it in self.items:
            p = it["poly"]
            (x0, y0), (x1, y1) = p.min(0), p.max(0)
            xs, ys = np.meshgrid(np.arange(x0, x1 + step, step), np.arange(y0, y1 + step, step))
            g = np.column_stack([xs.ravel(), ys.ravel()])
            out.append(g[inside(p, g)])
            out.append(p)
        return np.vstack(out) if out else np.zeros((0, 2))

    def state(self):
        return [{"name": it["name"], "conf": round(it["conf"], 3), "n": it["n"],
                 "cx": round(float(it["poly"][:, 0].mean()), 3), "cy": round(float(it["poly"][:, 1].mean()), 3),
                 "area": round(area(it["poly"]), 3), "full": it["full"],
                 "ground": it["ground"]} for it in self.items]
