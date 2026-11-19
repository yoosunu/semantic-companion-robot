"""hazards — 센서 사각의 바닥 위험물 판단 (11-19). 순수 계산 (ROS 없음) → tests/hazard_sim.py 로 시험.

  규칙 (11-19 Jasper):
    - 센서 (LiDAR, 나중에 ToF) 가 볼 수 있는 건 센서가 피한다. 비전은 이름표만.
    - 센서가 못 보는 것 (발닦개·양말처럼 LiDAR 17.5 cm 아래, 납작함) 은 비전만 볼 수 있다 → 비전이 costmap 에 찍는다.
    - 기본은 "안 밟기". 밀고 갈지는 나중 (길이 전혀 없을 때만).
  판단:
    ① 이름이 바닥형 (floor_types) — 임시 안전장치. 낮게 떠 있는 물체는 발자국이 멀리 엉뚱하게 찍힘 (11-19 그림)
       → 다음 단계: 바닥 감지 기록으로 "다가갈수록 끌려오나" (떠 있나) 를 재서 목록 의존을 줄임
    ② 그 모양 안 (+pad) 에 LiDAR 점이 scan_max 개 이하 → 센서가 못 봄 → 표시. 많으면 센서가 봄 → 이름표만
  모양: 납작한 것 (flat) = 마스크 윤곽 전체를 바닥에 내린 것의 볼록 껍질 (바닥에 깔려 있으니 윤곽 전체가 바닥 위)
        나머지 바닥형 (신발류) = 아래 발자국의 볼록 껍질 + min_w 두께
  같은 자리 (중심 merge m 안, 이름 무관) 는 합침: 모양 = **더 가까이서 본 것** (11-19 실측: 1.2 m 에서 본 발닦개 57×38 cm →
    69×59 cm. 마스크 가장자리 몇 px 가 먼 바닥에선 수 cm → 멀수록 크게 나옴. 합집합으로 하면 계속 커지기만 함), 점수 = 최대
  지우기: clear() (수동, /semantic/clear) 만. 자동 지우기는 다음 단계
"""
import math

import numpy as np

FLAT = ("doormat", "sock", "towel", "clothes", "plastic bag")
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

    def add(self, name, conf, poly, now, dist=9.9):
        """표시 더하기 (같은 자리면 합침) → "new" / "merged". dist = 본 거리 (가까울수록 모양이 정확)"""
        c = poly.mean(0)
        for it in self.items:
            if math.hypot(*(it["poly"].mean(0) - c)) < self.merge or inside(it["poly"], [c])[0] or inside(poly, [it["poly"].mean(0)])[0]:
                if dist < it["dist"]:
                    it["poly"], it["dist"] = poly, dist
                if conf >= it["conf"]:
                    it["name"], it["conf"] = name, conf
                it["t"], it["n"] = now, it["n"] + 1
                return "merged"
        self.items.append({"name": name, "conf": conf, "poly": poly, "t": now, "n": 1, "dist": dist})
        return "new"

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
                 "area": round(area(it["poly"]), 3)} for it in self.items]
