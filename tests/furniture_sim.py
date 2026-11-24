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


def real():
    """11-24 bed1 실물: 마스크 다시 돌림 → Furniture → 실측과 비교"""
    import cv2
    sys.path.insert(0, os.path.join(ROOT, "nodes", "perception"))
    sys.path.insert(0, os.path.join(ROOT, "tools", "lab"))
    import ground
    from yolo_onnx import Yolo
    import bed_snap as bs
    d = os.path.join(ROOT, "logs", "bed_set", "bed1")
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
            _, t = ground.cam_pose(pan, tilt)
            got = F.add_view(t[:2], bear, mf, scan, (0.0, 0.0))
            print(f"   pan {pan:+d}: bed {conf:.2f} 방향 {math.degrees(bear[0]):+.0f}°~{math.degrees(bear[1]):+.0f}° 다리 {[(round(x, 2), round(y, 2)) for x, y in got]}")
    r = F.shape()
    check("R1 실물: 다리 2 개 확정 (오른쪽 모서리 −0.29, 가운데 +0.67)", len(F.legs) == 2, [(round(l[0], 2), round(l[1], 2), l[2]) for l in F.legs])
    if r is None:
        check("R2 실물: 사각형", False, "None")
        return
    # 정답 (줄자 11-25): 다리 간격 97.5, 폭 108.5, 다리 6 개 (긴 변 3 개씩). 본 두 다리 = 오른쪽 모서리 + 가운데
    u, a, n = r["axis_u"], r["origin"], r["normal"]
    s = sorted(((np.array(l[:2]) - a) @ u) for l in F.legs)
    truth = [a + v * u + w * n for v in (s[0], s[1], s[1] + 0.975) for w in (0.0, 1.085)]
    miss = [tuple(np.round(t, 2)) for t in truth if not inside(r["corners"], t)]
    print(f"   사각형 (odom): {np.round(r['corners'], 2).tolist()}\n   길이 {r['hi'] - r['lo']:.2f} m, 깊이 {r['depth']:.2f} m, 간격 {r['spacing']:.2f} | {r['why']}")
    check("R2 실물: 다리 6 개 (본 2 + 정답 4) 전부 안", not miss, miss or "6/6")
    check("R3 실물: 오른쪽 (마스크 끝 = 다리) 은 모서리 다리 바깥 20 cm 안", 0 < s[0] - r["lo"] < 0.20, round(s[0] - r["lo"], 2))
    check("R4 실물: 깊이 = 벽까지 가정 (뒷줄 다리 1.085 ~ +25 cm)", 1.085 < r["depth"] < 1.335 and "벽까지" in r["why"]["depth"],
          (round(r["depth"], 2), r["why"]["depth"]))
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
    real()
print(f"\n{sum(results)}/{len(results)} 통과")
