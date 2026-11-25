#!/usr/bin/env python3
"""furniture_sim — 밑이 뜬 가구 영역 추론 (nodes/navigation/furniture.py v2) 시험 (11-25). ROS 없음.
   python3 tests/furniture_sim.py            가짜 방 시험
   python3 tests/furniture_sim.py --real     + 11-24 실물 기록 (logs/bed_set/bed1, 렌즈 보정 logs/bed_set/fisheye_640x480.json,
                                              models/yolo26s-seg.onnx 필요) → 위에서 본 그림 logs/bed_set/bed1/furniture_top.png
   가짜 방 = bed1 모양: 다리 6 개 (앞줄 x 0.9, 뒷줄 x 1.985, y −0.29 / +0.67 / +1.645), 왼쪽 끝 앞에 서랍장
"""
import json
import math
import os
import sys

import numpy as np

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
sys.path.insert(0, os.path.join(ROOT, "nodes", "navigation"))
import furniture as fu  # noqa: E402

results = []


def check(name, ok, info=""):
    results.append(bool(ok))
    print(("통과 " if ok else "실패 ") + name + (f"  ({info})" if info != "" else ""))


def inside(poly, p, tol=1e-9):
    """볼록 사각형 안인가"""
    s = [(poly[(i + 1) % 4] - poly[i])[0] * (p - poly[i])[1] - (poly[(i + 1) % 4] - poly[i])[1] * (p - poly[i])[0] for i in range(4)]
    return all(v >= -tol for v in s) or all(v <= tol for v in s)


YS = (-0.29, 0.67, 1.645)
FRONT = [(0.9, y) for y in YS]
BACK = [(1.985, y) for y in YS]
DRAWER = [((0.47, 1.19), (0.88, 1.19)), ((0.47, 1.19), (0.47, 1.75))]   # 서랍장 옆면·앞면 (11-24 bed1 LiDAR 모양)


def wall_at(x):
    return [((x, -3.0), (x, 3.0))]


def fake_scan(legs, segs, origin=(0.078, 0.0), r_leg=0.02):
    """origin 에서 360° 0.5° 간격 광선 → 점 (스캔 순서). legs = 원, segs = 선분"""
    o, pts = np.array(origin, float), []
    for a in np.arange(-math.pi, math.pi, math.radians(0.5)):
        d = np.array([math.cos(a), math.sin(a)])
        best = math.inf
        for p, q in segs:
            p, q = np.array(p, float), np.array(q, float)
            e = q - p
            den = d[0] * (-e[1]) + d[1] * e[0]
            if abs(den) < 1e-9:
                continue
            w = p - o
            t = (w[0] * (-e[1]) + w[1] * e[0]) / den
            v = (d[0] * w[1] - d[1] * w[0]) / den
            if t > 0 and 0 <= v <= 1:
                best = min(best, t)
        for lx, ly in legs:
            p = np.array([lx, ly]) - o
            t, s = p @ d, abs(p @ np.array([-d[1], d[0]]))
            if t > 0 and s < r_leg:
                best = min(best, t - math.sqrt(r_leg ** 2 - s * s))
        if best < 4.0:
            pts.append(o + best * d)
    return np.array(pts)


CAM = np.array([0.06, 0.0])
B = (math.radians(54), math.radians(-23))               # bed1 비전 방향 범위 (왼쪽 끝은 서랍장에 가려 +54°)
MASK = np.array([[0.9, y] for y in np.linspace(-0.33, 0.70, 30)])


def front_view(F, legs, segs, bearing=B, mask=MASK):
    return F.add_view(CAM, bearing, mask, fake_scan(legs, segs), (0, 0))


# 1. 앞에서만 봄, 침대가 벽에 붙음 (벽 x 2.03): 왼쪽 가려짐 → 다리 + 간격, 오른쪽 = 마스크 끝, 깊이 = 벽까지 (가정)
F = fu.Furniture()
got = front_view(F, FRONT + BACK, DRAWER + wall_at(2.03))
r = F.shape()
check("1 앞줄 다리 2 개 (왼쪽 다리는 서랍장 뒤, 뒷줄은 벽에 붙어 안 셈)", len(got) == 2, got)
ylo = lambda r: float(min(r["corners"][:, 1]))                  # lo/hi 는 축 따라 거리 (s) → 비교는 꼭짓점 y 로
yhi = lambda r: float(max(r["corners"][:, 1]))
check("1 오른쪽: 마스크 끝 = 다리 → 다리 바깥 15 cm 안", r and -0.45 < ylo(r) < -0.29 and "마스크 끝 = 다리" in r["why"]["lo"],
      (round(ylo(r), 2), r["why"]["lo"]))
check("1 왼쪽: 마스크가 더 감 → 가려진 다리 → +1.645 덮음", r and yhi(r) > 1.645 + 0.03 and "가려진" in r["why"]["hi"],
      (round(yhi(r), 2), r["why"]["hi"]))
check("1 깊이: 벽까지 가정 (1.13 근처)", r and 1.05 < r["depth"] < 1.2 and r["why"]["depth"] == "가정 (벽까지)" and not r["confirmed"],
      (round(r["depth"], 2), r["why"]["depth"]))
check("1 다리 6 개 전부 안", r and all(inside(r["corners"], np.array(p)) for p in FRONT + BACK))

# 2. 벽 없음 (열린 방) → 깊이 = 상한 2.2
F2 = fu.Furniture()
front_view(F2, FRONT[:2], DRAWER)
r2 = F2.shape()
check("2 벽 없음 → 깊이 상한 2.2", r2 and abs(r2["depth"] - 2.2) < 1e-6 and "상한" in r2["why"]["depth"], r2 and r2["why"]["depth"])

# 3. 벽에서 떨어진 침대 (벽 x 3.0): 앞에서 = 벽까지 가정 (2.1) → 오른쪽 옆에서 한 번 더 → 뒷줄 다리 + 옆면 마스크로 줄여 확정
F3 = fu.Furniture()
front_view(F3, FRONT + BACK, DRAWER + wall_at(3.0))
r3a = F3.shape()
check("3a 앞에서만 → 벽까지 가정 (2.1 근처)", r3a and 2.0 < r3a["depth"] < 2.2 and not r3a["confirmed"], (round(r3a["depth"], 2), r3a["why"]["depth"]))
side_cam = np.array([1.45, -1.1])
side_mask = np.array([[x, -0.29] for x in np.linspace(0.87, 2.02, 30)])
bear_side = (math.atan2(-0.29 - side_cam[1], 2.03 - side_cam[0]), math.atan2(-0.29 - side_cam[1], 0.85 - side_cam[0]))
got3 = F3.add_view(side_cam, bear_side, side_mask, fake_scan(FRONT + BACK, DRAWER + wall_at(3.0), origin=side_cam), side_cam)
r3 = F3.shape()
check("3b 옆에서 → 뒷줄 다리 찾음", any(abs(x - 1.985) < 0.05 for x, _ in got3), got3)
check("3b 옆에서 → 깊이 확정, 줄어듦 (1.085 + pad 근처)", r3 and r3["confirmed"] and 1.08 < r3["depth"] < 1.20,
      (round(r3["depth"], 2), r3["why"]["depth"]))
check("3b 줄어든 뒤에도 다리 6 개 전부 안", r3 and all(inside(r3["corners"], np.array(p)) for p in FRONT + BACK))

# 4. 식탁 같은 것: 다리 2 개, 마스크가 양쪽 다 다리에서 끝남 → 늘리지 않음
F4 = fu.Furniture(name="dining table")
legs4 = [(0.9, -0.3), (0.9, 0.5)]
b4 = (math.atan2(0.52, 0.84), math.atan2(-0.32, 0.84))
front_view(F4, legs4, wall_at(2.5), bearing=b4, mask=np.array([[0.9, y] for y in np.linspace(-0.32, 0.52, 20)]))
r4 = F4.shape()
check("4 마스크 끝 = 다리 양쪽 → 늘리지 않음 (다리 바깥 15 cm 안)", r4 and -0.45 < ylo(r4) < -0.3 and 0.5 < yhi(r4) < 0.65,
      r4 and (round(ylo(r4), 2), round(yhi(r4), 2), r4["why"]["lo"], r4["why"]["hi"]))

# 5. 다리 1 개 → 영역 없음
F5 = fu.Furniture()
front_view(F5, [FRONT[0]], wall_at(2.03), mask=np.array([[0.9, y] for y in np.linspace(-0.33, 0.1, 10)]))
check("5 다리 1 개 → None", F5.shape() is None, len(F5.legs))

# 6. 마스크에서 먼 작은 덩어리 (의자 다리) → 침대 다리 아님
F6 = fu.Furniture()
front_view(F6, FRONT[:2] + [(0.5, -0.8)], wall_at(2.03))
check("6 마스크 밖 의자 다리 → 안 씀", len(F6.legs) == 2, [tuple(round(v, 2) for v in l[:2]) for l in F6.legs])

# 7. 같은 자리에서 두 번 → 같은 다리로 합침, 증거 없으니 깊이는 그대로 가정
F7 = fu.Furniture()
for _ in range(2):
    front_view(F7, FRONT + BACK, DRAWER + wall_at(2.03))
r7 = F7.shape()
check("7 두 번 본 다리 → 합침, 깊이 여전히 가정", len(F7.legs) == 2 and all(l[2] == 2 for l in F7.legs) and not r7["confirmed"],
      ([l[2] for l in F7.legs], r7["why"]["depth"]))

# 8. 상한: 가려진 쪽이 너무 길면 가려진 쪽부터 자름
F8 = fu.Furniture(max_len=1.5)
front_view(F8, FRONT + BACK, DRAWER + wall_at(2.03))
r8 = F8.shape()
check("8 상한 1.5 → 길이 1.5, 오른쪽 (마스크 끝 = 다리) 그대로", r8 and abs((r8["hi"] - r8["lo"]) - 1.5) < 1e-6 and -0.45 < ylo(r8) < -0.29,
      r8 and (round(r8["hi"] - r8["lo"], 2), round(ylo(r8), 2)))

# 9. 비스듬히 (카메라가 오른쪽 끝 바깥 앞, 11-25 bed2 상황) → 그래도 앞면 비전 끝 씀 (전엔 "비전 없음" → 양쪽 1 m 씩 → 상한)
F9 = fu.Furniture()
cam9 = np.array([0.3, -0.9])
b9 = (math.atan2(1.69 - cam9[1], 0.9 - cam9[0]), math.atan2(-0.33 - cam9[1], 0.9 - cam9[0]))
F9.add_view(cam9, b9, np.array([[0.9, y] for y in np.linspace(-0.33, 1.69, 40)]), fake_scan(FRONT + BACK, wall_at(2.03), origin=cam9), cam9)
r9 = F9.shape()
check("9 비스듬히 → 오른쪽 = 옆면 보임, 왼쪽 끝 광선은 앞줄을 13° 로 스침 → 거리 안 씀 (넉넉하게), 다리 6 개 안",
      r9 and "옆면 보임" in r9["why"]["lo"] and "스침" in r9["why"]["hi"] and -0.45 < ylo(r9) < -0.29
      and all(inside(r9["corners"], np.array(p)) for p in FRONT + BACK), r9 and (round(ylo(r9), 2), round(yhi(r9), 2), r9["why"]["lo"], r9["why"]["hi"]))

# 10. 비스듬히 + 마스크 오른쪽 끝 = 옆면 먼 모서리 (bed2 실물 모양) → 오른쪽 늘리지 않음, 옆면 → 깊이 확정 (벽 넘으면 벽에서 자름),
#     반대쪽 숨은 다리 (서랍장 뒤) 는 그대로 덮음 (전엔 오른쪽 31 cm "가려진 다리" → 상한에서 반씩 잘려 왼쪽 숨은 다리가 빠짐)
F10 = fu.Furniture()
b10 = (math.atan2(1.20 - cam9[1], 0.9 - cam9[0]), math.atan2(-0.33 - cam9[1], 1.95 - cam9[0]))   # 왼쪽 = 서랍장에서 끊김, 오른쪽 = 옆면 먼 모서리
F10.add_view(cam9, b10, np.array([[0.9, y] for y in np.linspace(-0.33, 0.70, 30)]), fake_scan(FRONT + BACK, DRAWER + wall_at(2.03), origin=cam9), cam9)
r10 = F10.shape()
check("10 비스듬히 옆면 → 오른쪽 = 다리 + pad, 왼쪽 숨은 다리 덮음, 다리 6 개 안", r10 and "옆면 보임" in r10["why"]["lo"]
      and -0.40 < ylo(r10) < -0.29 and all(inside(r10["corners"], np.array(p)) for p in FRONT + BACK), r10 and (round(ylo(r10), 2), round(yhi(r10), 2), r10["why"]))
check("10 옆면 광선이 옆면 선을 19° 로 스침 → 깊이 증거 버림 → 벽까지 가정 (1.0~1.2)", r10 and not r10["confirmed"] and "스침" in r10["why"]["depth"]
      and 1.0 < r10["depth"] < 1.2, r10 and (round(r10["depth"], 2), r10["why"]["depth"]))

# 12. 화면 끝에 잘린 마스크 끝 → 방향 증거 아님 (11-25 bed2: 잘린 끝을 침대 끝으로 믿어 숨은 다리가 빠짐)
F12 = fu.Furniture()
front_view(F12, FRONT + BACK, DRAWER + wall_at(2.03), bearing=(None, math.radians(-23)))   # 왼쪽 끝이 화면에서 잘림
r12 = F12.shape()
check("12 왼쪽 잘림 → 비전 없음 → 넉넉하게 (숨은 다리 +1.645 덮음)", r12 and "비전 없음" in r12["why"]["hi"] and yhi(r12) > 1.665,
      r12 and (round(yhi(r12), 2), r12["why"]["hi"]))

# 11. 교차 각도: 옆에서 가파르게 (54°) 보면 그대로 확정 (시험 3 과 같은 장면, 스침 규칙이 좋은 증거는 안 버림)
check("11 가파른 옆면 (54°) → 확정 유지", r3 and r3["confirmed"] and "스침" not in r3["why"]["depth"], r3 and r3["why"]["depth"])

# 13~16. FurnitureMap (floor_hazard 가 씀): 같은 침대 여러 번 → 하나, 멀리 다른 침대 → 둘, 점수 낮음 → 안 씀, costmap 점이 사각형을 채움
M = fu.FurnitureMap()
scan1 = fake_scan(FRONT + BACK, DRAWER + wall_at(2.03))
M.observe("bed", 0.76, CAM, B, MASK, scan1, (0, 0))
M.observe("bed", 0.79, CAM, B, MASK, scan1, (0, 0))
check("13 같은 침대 두 번 → 하나", len(M.items) == 1 and len(M.items[0].views) == 2, len(M.items))
far_legs = [(4.0, 3.0), (4.0, 4.0)]
_, _, how = M.observe("bed", 0.7, CAM, (math.atan2(4.05, 3.94), math.atan2(2.95, 3.94)), np.array([[4.0, y] for y in np.linspace(2.95, 4.05, 20)]),
                      fake_scan(far_legs, wall_at(5.0)), (0, 0))
check("14 멀리 다른 침대 → 새로 (둘)", how == "new" and len(M.items) == 2, (how, len(M.items)))
F_, _, how = M.observe("bed", 0.25, CAM, B, MASK, scan1, (0, 0))
check("15 점수 0.25 (< 0.3) → 안 씀", F_ is None and how.startswith("skip"), how)
F_, _, how = M.observe("bed", 0.37, CAM, B, MASK, scan1, (0, 0))
check("15b 점수 0.37 + 기존 침대 자리 → 받음 (이중 문턱)", F_ is M.items[0] and how.startswith("same") and "낮은 점수" in how and len(M.items) == 2, how)
F_, _, how = M.observe("bed", 0.37, CAM, (math.atan2(-2.95, 3.94), math.atan2(-4.05, 3.94)), np.array([[4.0, -y] for y in np.linspace(2.95, 4.05, 20)]),
                       fake_scan([(4.0, -3.0), (4.0, -4.0)], wall_at(5.0)), (0, 0))
check("15c 점수 0.37 + 처음 보는 자리 → 안 씀 (새 가구는 0.5)", F_ is None and "새 가구" in how and len(M.items) == 2, how)
cp = M.cloud_points()
r0 = M.items[0].shape()
check("16 costmap 점: 첫 침대 다리 6 개 자리 둘레 3 cm 안에 점 있음, 사각형 밖 점 없음",
      all(np.hypot(*(cp - p).T).min() < 0.03 for p in FRONT + BACK)
      and all(inside(r0["corners"], q, tol=1e-6) or inside(M.items[1].shape()["corners"], q, tol=1e-6) for q in cp if M.items[1].shape() is not None),
      len(cp))
M.clear()
check("16 지우기 → 0", not M.items and len(M.cloud_points()) == 0)

# 17. ground.pixel_bearing: 바닥 점을 픽셀로 보낸 뒤 방향으로 되돌리면 렌즈에서 그 점 방향과 같음 (렌즈 값은 가짜, 식 검증용)
try:
    sys.path.insert(0, os.path.join(ROOT, "nodes", "perception"))
    import ground as gr
    K = np.array([[260.0, 0, 320], [0, 260.0, 240], [0, 0, 1]]); D = np.zeros((4, 1))
    errs = []
    for pan, tilt in ((0, -15), (-40, -15), (30, -5)):
        pts = np.array([[0.9, 0.3], [1.2, -0.5], [0.7, 0.0]])
        px = gr.floor_to_pixels(pts, K, D, pan, tilt)
        ok = np.isfinite(px).all(1)
        _, t = gr.cam_pose(pan, tilt)
        want = np.arctan2(pts[ok, 1] - t[1], pts[ok, 0] - t[0])
        errs += list(np.degrees(np.abs(gr.pixel_bearing(px[ok], K, D, pan, tilt) - want)))
    check("17 pixel_bearing 되돌리기 오차 < 0.1°", errs and max(errs) < 0.1, round(max(errs), 4) if errs else "점 없음")
except ImportError as e:
    print(f"건너뜀 17 (ground 못 불러옴: {e})")

# 22. 같은 다리를 다른 각도에서 10 cm 어긋나게 (11-25 20:00 실기: 다리 4 개, 간격 0.10) → 합쳐지고 간격 0.96
F22 = fu.Furniture()
front_view(F22, FRONT + BACK, DRAWER + wall_at(2.03))
F22.add_view(CAM, B, MASK + [0.0, 0.10], fake_scan([(0.9, -0.19), (0.9, 0.77)] + BACK, DRAWER + wall_at(2.03)), (0, 0))
r22 = F22.shape()
check("22 같은 다리 10 cm 어긋남 → 다리 2 개로 합침, 간격 0.9~1.0", len(F22.legs) == 2 and r22 and 0.9 < r22["spacing"] < 1.0,
      (len(F22.legs), r22 and round(r22["spacing"], 2)))

# 18~21. 회전의자 (11-25 chair1: 기둥 = LiDAR 작은 덩어리, 반지름 = 장면 중간값들의 중간값, 줄자 별 다리 지름 68 cm)
COL = (1.0, 0.15)
def chair_foot(rng, r_wheel=0.32, frac_float=0.35, n=60, arc=(-1.2, 1.2)):
    """허브·다리·바퀴 쪽 (기둥에서 0.08 ~ 바퀴, 바닥 — chair1 실물: 기둥 ↔ 가장 가까운 마스크 아래점 8~13 cm)
       + 떠 있는 좌석 옆 (0.45~0.56, 멀리 찍힘), 로봇 쪽 반원만 보임"""
    a = rng.uniform(*arc, n) + math.pi                              # 로봇 (원점) 쪽
    u = rng.random(n)                                               # 아래에서 보면 별 다리 끝·바퀴가 대부분, 허브는 조금
    rr = np.where(u < frac_float, rng.uniform(0.45, 0.56, n),
                  np.where(u < frac_float + 0.07, rng.uniform(0.08, 0.12, n), rng.uniform(max(0.08, r_wheel - 0.06), r_wheel + 0.02, n)))
    return np.column_stack([COL[0] + rr * np.cos(a), COL[1] + rr * np.sin(a)])
rng = np.random.default_rng(1)
Mc = fu.FurnitureMap()
for _ in range(3):
    Mc.observe("chair", 0.6, CAM, (math.radians(30), math.radians(-20)), chair_foot(rng), fake_scan([COL], wall_at(2.5)), (0, 0))
rc = Mc.items[0].shape() if Mc.items else None
check("18 의자 3 장 → 원 하나, 중심 = 기둥 (3 cm 안), 반지름 0.34~0.42 (떠 있는 점 섞여도 중간값)",
      len(Mc.items) == 1 and rc and np.hypot(*(rc["center"] - COL)) < 0.03 and 0.34 <= rc["radius"] <= 0.42,
      rc and (np.round(rc["center"], 2).tolist(), round(rc["radius"], 3), rc["why"]["lo"]))
M1 = fu.FurnitureMap()
M1.observe("chair", 0.6, CAM, (math.radians(30), math.radians(-20)), chair_foot(rng, r_wheel=0.16, frac_float=0.0), fake_scan([COL], wall_at(2.5)), (0, 0))
r1 = M1.items[0].shape()
check("19 한 장에 바퀴가 가까이만 보임 (중간값 작음) → 최소 0.28 + 5 cm", abs(r1["radius"] - 0.33) < 1e-6 and "최소" in r1["why"]["lo"],
      (round(r1["radius"], 3), r1["why"]["lo"]))
NEW = (0.6, -0.6)
foot2 = np.vstack([np.column_stack([NEW[0] + 0.3 * np.cos(np.linspace(2, 4.3, 30)), NEW[1] + 0.3 * np.sin(np.linspace(2, 4.3, 30))]),
                   [[NEW[0] - 0.1, NEW[1] + 0.02]]])                 # 허브 점 하나 (기둥 짝짓기)
Mc.observe("chair", 0.6, CAM, (math.radians(30), math.radians(-50)), foot2, fake_scan([NEW], wall_at(2.5)), (0, 0))
check("20 의자를 옮김 (보이는 범위인데 옛 기둥 없음) → 옛 원 지우고 새 자리 하나",
      len(Mc.items) == 1 and np.hypot(*(Mc.items[0].shape()["center"] - NEW)) < 0.05, [np.round(i.shape()["center"], 2).tolist() for i in Mc.items])
Mb = fu.FurnitureMap()
Mb.observe("bed", 0.76, CAM, B, MASK, fake_scan(FRONT + BACK, DRAWER + wall_at(2.03)), (0, 0))
Mb.observe("chair", 0.6, CAM, (math.radians(-35), math.radians(-60)), foot2, fake_scan([NEW], DRAWER + wall_at(2.03)), (0, 0))
check("21 침대 + 의자 → 둘 다 있음 (사각형 + 원), costmap 점에 둘 다",
      sorted(i.name for i in Mb.items) == ["bed", "chair"] and len(Mb.cloud_points()) > 1000, [i.name for i in Mb.items])


# 23~26. 11-25 밤 의자 2차 (Jasper: 기둥을 LiDAR 로 봐야만 그리면 4 다리 의자·가려진 기둥은 영영 못 그림)
#   접점 = 끝 방향 사이 + 마스크 아래점 15 cm 안 LiDAR 점, 폭 = 끝 방향을 접점 거리에, 뒤쪽 = 보인 폭만큼 가정 (벽에서 멈춤)
def bearings(cam, pts):
    a = np.arctan2(np.asarray(pts)[:, 1] - cam[1], np.asarray(pts)[:, 0] - cam[0])
    return (float(a.max()), float(a.min()))
def covers(poly, pts, r=0.02):
    return all(fu._inside(poly, np.array(p) + r * np.array(d)) for p in pts for d in ((1, 0), (-1, 0), (0, 1), (0, -1)))
LEG4 = [(0.85, 0.20), (0.85, -0.20), (1.25, 0.20), (1.25, -0.20)]
SEAT = [(0.82, 0.23), (0.82, -0.23), (1.28, 0.23), (1.28, -0.23)]
rng4 = np.random.default_rng(4)
foot4 = np.vstack([[(0.85 + rng4.normal(0, 0.01), 0.20 + rng4.normal(0, 0.01)) for _ in range(4)],
                   [(0.85 + rng4.normal(0, 0.01), -0.20 + rng4.normal(0, 0.01)) for _ in range(4)],
                   np.column_stack([rng4.uniform(1.5, 2.0, 12), rng4.uniform(-0.18, 0.18, 12)])])   # 앞 가로대 (떠 있어 멀리)
M4 = fu.FurnitureMap()
_, got, how = M4.observe("chair", 0.7, CAM, bearings(CAM, SEAT), foot4, fake_scan(LEG4, wall_at(2.5)), (0, 0))
r4 = M4.items[0].shape() if M4.items else None
check("23 4 다리 의자 앞에서 (앞다리 2 개만 마스크와 맞음) → 2차, 다리 4 개 다 덮음, 폭 < 0.65 깊이 < 0.65",
      r4 is not None and r4.get("method") == 2 and covers(r4["corners"], LEG4) and r4["hi"] < 0.65 and r4["depth"] < 0.65,
      r4 and (r4["why"]["lo"], round(r4["hi"], 2), round(r4["depth"], 2), r4["miss"]))
CAMS = np.array([1.05, -0.95])                                       # 옆 (오른쪽) 에서
footS = np.vstack([[(0.85 + rng4.normal(0, 0.01), -0.20 + rng4.normal(0, 0.01)) for _ in range(4)],
                   [(1.25 + rng4.normal(0, 0.01), -0.20 + rng4.normal(0, 0.01)) for _ in range(4)]])
M4.observe("chair", 0.7, CAMS, bearings(CAMS, SEAT), footS, fake_scan(LEG4, wall_at(2.5), origin=CAMS), (1.05, -1.0))
r4b = M4.items[0].shape()
check("24 옆에서 한 번 더 → 같은 의자, 여전히 4 다리 덮음, 깊이 0.40~0.60 (보인 깊이가 늘고 가정은 줄어듦)",
      len(M4.items) == 1 and covers(r4b["corners"], LEG4) and 0.40 <= r4b["depth"] <= 0.60,
      (len(M4.items), r4b["why"]["lo"], round(r4b["depth"], 2)))
rng5 = np.random.default_rng(5)
NEARWALL = [((1.10, -0.6), (1.10, 0.9))]                             # 기둥 뒤 8 cm 에 벽·책상 옆판 → 떨어짐 조건 실패
B5 = (math.radians(30), math.radians(-12))
M5 = fu.FurnitureMap()
def chair_foot2(rng, n=60):
    """물리적으로 맞는 발자국: 바퀴 (바닥, 로봇 쪽 반원 0.28~0.34) + 허브 + 떠 있는 좌석 가장자리 (실제보다 멀리 찍힘 → 기둥 뒤쪽)"""
    a = rng.uniform(-1.2, 1.2, n)
    u = rng.random(n)
    side = np.where(u < 0.35, 0.0, math.pi)                          # 떠 있는 점은 뒤 (+x 쪽), 나머지는 앞 (로봇 쪽)
    rr = np.where(u < 0.35, rng.uniform(0.10, 0.30, n), np.where(u < 0.42, rng.uniform(0.08, 0.12, n), rng.uniform(0.28, 0.34, n)))
    return np.column_stack([COL[0] + rr * np.cos(a + side), COL[1] + rr * np.sin(a + side)])
for _ in range(2):
    M5.observe("chair", 0.8, CAM, B5, chair_foot2(rng5), fake_scan([COL], NEARWALL), (0, 0))
r5 = M5.items[0].shape() if M5.items else None
check("25 회전의자 기둥이 벽에 붙음 (1차 실패, 이유 '붙어 있음') → 2차로 그림, 기둥·앞쪽 바퀴 덮음, 벽 안 넘음",
      r5 is not None and r5.get("method") == 2 and "붙어" in r5["miss"] and covers(r5["corners"], [COL, (COL[0] - 0.30, COL[1])])
      and r5["corners"][:, 0].max() <= 1.10 + 0.06, r5 and (r5["why"]["lo"], r5["miss"], round(float(r5["corners"][:, 0].max()), 2)))
M6 = fu.FurnitureMap()
M6.observe("chair", 0.6, CAM, B5, chair_foot2(rng5), fake_scan([COL], wall_at(2.5)), (0, 0))
r6 = M6.items[0].shape()
check("26 기둥 보이면 그대로 1차 (원), 2차는 alt 로 같이 (비교용)", r6["method"] == 1 and r6["alt"] is not None and r6["alt"]["method"] == 2,
      (r6["why"]["lo"], r6["alt"] and r6["alt"]["why"]["lo"]))


def real_chair(set_name="chair1"):
    """11-25 chair1 실물 (glance 와 같은 발자국: 3 열마다, 3 m 안) → 기둥 (+1.02, +0.15), 줄자 반지름 0.34"""
    import cv2
    sys.path.insert(0, os.path.join(ROOT, "nodes", "perception"))
    sys.path.insert(0, os.path.join(ROOT, "tools", "lab"))
    import ground
    from yolo_onnx import Yolo
    import bed_snap as bs
    d = os.path.join(ROOT, "logs", "bed_set", set_name)
    print(f"--- 실물 {set_name}")
    K, D, _ = ground.load_calib(os.path.join(ROOT, "logs", "bed_set", "fisheye_640x480.json"))
    yolo = Yolo(os.path.join(ROOT, "models", "yolo26s-seg.onnx"), size=320, conf=0.2)
    M = fu.FurnitureMap()
    for p in json.load(open(os.path.join(d, "poses.json")))["poses"]:
        bgr = cv2.imread(os.path.join(d, p["file"]))
        scan = bs.scan_to_base(p["scan"])
        for name, conf, bx, by, bw, bh, mask in [x for x in yolo.detect(bgr, masks=True) if x[0] == "chair" and len(x) == 7]:
            cols = np.nonzero(mask.any(0))[0]
            px = [[c + 0.5, np.nonzero(mask[:, c])[0].max() + 0.5] for c in cols[::3]]
            fl = ground.pixels_to_floor(px, K, D, p["pan"], p["tilt"])
            fl = fl[np.isfinite(fl).all(1)]
            fl = fl[np.hypot(fl[:, 0], fl[:, 1]) < 3.0]
            b = ground.pixel_bearing([px[0], px[-1]], K, D, p["pan"], p["tilt"])
            cut = (cols.min() <= 3, cols.max() >= mask.shape[1] - 4)
            _, t = ground.cam_pose(p["pan"], p["tilt"])
            F, got, how = M.observe("chair", conf, t[:2], [None if c else float(v) for v, c in zip(b, cut)], fl, scan, (0.0, 0.0))
            print(f"   pan {p['pan']:+d}: chair {conf:.2f} → {how} 기둥 {[(round(x, 2), round(y, 2)) for x, y in got]}")
    chairs = [i for i in M.items if i.name == "chair"]
    r = chairs[0].shape() if chairs else None
    check(f"R1 {set_name}: 의자 하나, 중심 = 기둥 (+1.02, +0.15) 3 cm 안", len(chairs) == 1 and r is not None
          and np.hypot(*(r["center"] - [1.02, 0.15])) < 0.03, r and (np.round(r["center"], 3).tolist(), len(chairs)))
    check(f"R2 {set_name}: 반지름 = 줄자 0.34 + pad 5 cm (±4 cm)", r is not None and abs(r["radius"] - 0.39) <= 0.04,
          r and (round(r["radius"], 3), r["why"]["lo"]))
    a2 = chairs[0].shape2() if chairs else None
    if a2 is not None:                                               # 11-25 밤 2차를 같은 장면으로 (기둥 안 쓰고): 줄자 별 다리 지름 0.68, 기둥 (1.02, 0.15)
        P = a2["corners"]
        print(f"   2차: {a2['why']['lo']} | x {P[:, 0].min():+.2f}~{P[:, 0].max():+.2f} y {P[:, 1].min():+.2f}~{P[:, 1].max():+.2f}")
    check(f"R4 {set_name}: 2차만으로도 기둥 + 앞쪽 바퀴 (기둥 − 0.34) 덮음", a2 is not None and covers(a2["corners"], [(1.02, 0.15), (0.70, 0.15)]),
          a2 and np.round(a2["corners"], 2).tolist())


def real(set_name="bed1"):
    """실물 기록 (11-24 bed1 정면, 11-25 bed2 오른쪽으로 옮겨 비스듬히·옆면 보임): 마스크 다시 돌림 → Furniture → 실측과 비교.
       bed3 (11-25 03:19): 더 오른쪽·안쪽, 오른쪽 짧은 면을 최대한 정면으로. 알고리즘 안 고치고 검증용.
       정답: 셋 다 본 두 다리 = 오른쪽 모서리 + 가운데 (같은 침대, 줄자 간격 97.5 · 폭 108.5) → 왼쪽 모서리 다리는 서랍장 뒤"""
    import cv2
    sys.path.insert(0, os.path.join(ROOT, "nodes", "perception"))
    sys.path.insert(0, os.path.join(ROOT, "tools", "lab"))
    import ground
    from yolo_onnx import Yolo
    import bed_snap as bs
    d = os.path.join(ROOT, "logs", "bed_set", set_name)
    print(f"--- 실물 {set_name}")
    K, D, _ = ground.load_calib(os.path.join(ROOT, "logs", "bed_set", "fisheye_640x480.json"))
    yolo = Yolo(os.path.join(ROOT, "models", "yolo26s-seg.onnx"), size=320, conf=0.2)
    F = fu.Furniture()
    poses = json.load(open(os.path.join(d, "poses.json")))["poses"]
    for p in poses:
        bgr = cv2.imread(os.path.join(d, p["file"]))
        pan, tilt = p["pan"], p["tilt"]
        scan = bs.scan_to_base(p["scan"])
        for name, conf, bx, by, bw, bh, mask in [x for x in yolo.detect(bgr, masks=True) if x[0] == "bed" and x[1] >= 0.5 and len(x) == 7]:
            mf = ground.pixels_to_floor(bs.mask_bottom(mask), K, D, pan, tilt)
            cols = np.where(mask.any(0))[0]
            bear = [math.radians(bs.ray_bearing([c, mask[:, c].nonzero()[0].max()], K, D, pan, tilt)) for c in (cols.min(), cols.max())]
            cut = (cols.min() <= 3, cols.max() >= mask.shape[1] - 4)       # 화면 끝에 잘린 쪽 → 방향 증거 아님
            bear = [None if c else b for b, c in zip(bear, cut)]
            _, t = ground.cam_pose(pan, tilt)
            got = F.add_view(t[:2], bear, mf, scan, (0.0, 0.0))
            fmt = lambda v: "잘림" if v is None else f"{math.degrees(v):+.0f}°"
            print(f"   pan {pan:+d}: bed {conf:.2f} 방향 {fmt(bear[0])}~{fmt(bear[1])} 다리 {[(round(x, 2), round(y, 2)) for x, y in got]}")
    r = F.shape()
    check(f"R1 {set_name}: 다리 2 개 확정 (오른쪽 모서리 + 가운데)", len(F.legs) == 2, [(round(l[0], 2), round(l[1], 2), l[2]) for l in F.legs])
    if r is None:
        check(f"R2 {set_name}: 사각형", False, "None")
        return
    # 정답 (줄자 11-25): 다리 간격 97.5, 폭 108.5, 다리 6 개 (긴 변 3 개씩). 본 두 다리 = 오른쪽 모서리 + 가운데
    u, a, n = r["axis_u"], r["origin"], r["normal"]
    s = sorted(((np.array(l[:2]) - a) @ u) for l in F.legs)
    truth = [a + v * u + w * n for v in (s[0], s[1], s[1] + 0.975) for w in (0.0, 1.085)]
    miss = [tuple(np.round(t, 2)) for t in truth if not inside(r["corners"], t)]
    print(f"   사각형 (odom): {np.round(r['corners'], 2).tolist()}\n   길이 {r['hi'] - r['lo']:.2f} m, 깊이 {r['depth']:.2f} m, 간격 {r['spacing']:.2f} | {r['why']}")
    check(f"R2 {set_name}: 다리 6 개 (본 2 + 정답 4) 전부 안", not miss, miss or "6/6")
    check(f"R3 {set_name}: 오른쪽 끝은 모서리 다리 바깥 20 cm 안 (안 늘림)", 0 < s[0] - r["lo"] < 0.20, (round(s[0] - r["lo"], 2), r["why"]["lo"]))
    check(f"R4 {set_name}: 깊이 = 뒷줄 다리 1.085 ~ +25 cm", 1.085 < r["depth"] < 1.335, (round(r["depth"], 2), r["why"]["depth"]))
    if set_name == "bed2":
        check("R5 bed2: 옆면 먼 모서리 광선 (8° 스침) → 깊이 증거로 안 씀", "스침" in r["why"]["depth"] and not r["confirmed"], r["why"]["depth"])
    S, img = 150, np.full((420, 640, 3), 255, np.uint8)
    to = lambda q: (int(400 - q[1] * S), int(380 - q[0] * S))
    for x, y in bs.scan_to_base(poses[2]["scan"]):
        cv2.circle(img, to((x, y)), 1, (150, 150, 150), -1)
    cv2.polylines(img, [np.array([to(q) for q in r["corners"]], np.int32)], True, (0, 0, 220), 2)
    for t in truth:
        cv2.circle(img, to(t), 6, (0, 150, 0), 1)
    for l in F.legs:
        cv2.circle(img, to(l[:2]), 6, (0, 150, 0), -1)
    cv2.rectangle(img, to((0.126, 0.13)), to((-0.14, -0.13)), (90, 90, 90), 1)
    cv2.putText(img, "red = blocked area, filled = LiDAR legs, ring = true legs (tape)", (8, 18), 0, 0.45, (0, 0, 0), 1)
    cv2.imwrite(os.path.join(d, "furniture_top.png"), img)
    print(f"   그림: {os.path.join(d, 'furniture_top.png')}")


if "--real" in sys.argv:
    for name in ("bed1", "bed2", "bed3"):
        if os.path.isdir(os.path.join(ROOT, "logs", "bed_set", name)):
            real(name)
    if os.path.isdir(os.path.join(ROOT, "logs", "bed_set", "chair1")):
        real_chair("chair1")
print(f"\n{sum(results)}/{len(results)} 통과")
