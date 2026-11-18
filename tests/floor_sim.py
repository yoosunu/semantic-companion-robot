#!/usr/bin/env python3
"""floor_detect + ground 시뮬레이션 (ROS 없이). python3 tests/floor_sim.py
   가짜 바닥 사진 (11-18 Pi 렌즈 보정 K·D, 고개 0,−20) 에 바닥 위 물건을 그려 넣고: 감지·물건이 시작하는 거리·1.5 m 밖 무시"""
import os
import sys

import cv2
import numpy as np

R = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "nodes", "perception")
sys.path.insert(0, R)
import floor_detect  # noqa: E402
import ground  # noqa: E402

K = np.array([[319.9, 0, 321.0], [0, 320.1, 224.4], [0, 0, 1]])
D = np.array([-0.0229, -0.0288, 0.0251, -0.0102]).reshape(4, 1)
PAN, TILT = 0.0, -20.0
results = []


def check(name, ok, info=""):
    results.append(bool(ok))
    print(f"{'OK ' if ok else 'XX '} {name} {info}")


def frame(objs, seed=0, glare=False):
    rng = np.random.default_rng(seed)
    img = np.full((480, 640, 3), (170, 190, 205), np.uint8)               # 바닥 (BGR 베이지)
    if glare:                                                              # 11-18 실물: 먼 바닥이 밝고 색 바램 (55 cm~)
        ys, xs = np.mgrid[0:480, 0:640]
        fl = ground.pixels_to_floor(np.column_stack([xs.ravel(), ys.ravel()]), K, D, PAN, TILT)
        d = np.nan_to_num(np.hypot(fl[:, 0], fl[:, 1]), nan=9).reshape(480, 640)
        k = np.clip((d - 0.5) / 0.6, 0, 1)[..., None]
        img = (img * (1 - k) + np.array([225, 228, 232]) * k).astype(np.uint8)
    img = np.clip(img + rng.normal(0, 4, img.shape), 0, 255).astype(np.uint8)
    for x0, x1, y0, y1 in objs:                                            # 바닥 위 사각형 물건 (m)
        xs, ys = np.linspace(x0, x1, 12), np.linspace(y0, y1, 12)
        edge = [(x, y0) for x in xs] + [(x1, y) for y in ys] + [(x, y1) for x in xs[::-1]] + [(x0, y) for y in ys[::-1]]
        px = ground.floor_to_pixels(np.array(edge), K, D, PAN, TILT)
        px = px[np.isfinite(px).all(1)].astype(np.int32)
        cv2.fillPoly(img, [px], (40, 40, 45))
    return img


fd = floor_detect.FloorDetect(K, D, (640, 480))
for k in range(5):                                                         # 빈 바닥으로 기준 배우기
    b, odd, _ = fd.step(frame([], k), PAN, TILT)
check("1 빈 바닥: 후보 없음", not b, [x["near"] for x in b])

b, _, _ = fd.step(frame([(0.60, 0.70, -0.05, 0.05)]), PAN, TILT)
check("2 0.60 m 물건: 시작 거리", len(b) == 1 and abs(b[0]["near"] - 0.60) < 0.03, [round(x["near"], 3) for x in b])

b, _, _ = fd.step(frame([(1.20, 1.35, 0.10, 0.25)]), PAN, TILT)
check("3 1.20 m 왼쪽 물건", len(b) == 1 and abs(np.hypot(1.20, 0.10) - b[0]["near"]) < 0.08 and b[0]["pts"][:, 1].mean() > 0.05,
      [(round(x["near"], 3), round(float(x["pts"][:, 1].mean()), 2)) for x in b])

b, _, _ = fd.step(frame([(2.0, 2.3, -0.1, 0.1)]), PAN, TILT)
check("4 2 m 물건: 1.5 m 밖 무시", not b, [round(x["near"], 2) for x in b])

b, _, _ = fd.step(frame([(0.45, 0.55, -0.20, -0.10), (0.9, 1.0, 0.05, 0.15)]), PAN, TILT)
check("5 두 개: 가까운 순", len(b) == 2 and b[0]["near"] < b[1]["near"] and b[0]["pts"][:, 1].mean() < 0,
      [round(x["near"], 3) for x in b])

# 6. 먼 바닥 반사 (11-18 실물): 거리대별 기준이면 띠 없음, 물건은 잡음 / 한 기준만 (bands 하나) 이면 띠가 생김 (비교)
fd = floor_detect.FloorDetect(K, D, (640, 480))
for k in range(8):
    fd.step(frame([], k, glare=True), PAN, TILT)
b, _, _ = fd.step(frame([], 9, glare=True), PAN, TILT)
check("6a 먼 바닥 반사: 거리대별 기준 → 띠 없음", not b, [(round(x["near"], 2), x["area"]) for x in b])
b, _, _ = fd.step(frame([(0.90, 1.00, -0.05, 0.05)], 10, glare=True), PAN, TILT)
check("6b 반사 바닥 위 0.9 m 물건은 잡음", len(b) == 1 and abs(b[0]["near"] - 0.90) < 0.05, [round(x["near"], 3) for x in b])
one = floor_detect.FloorDetect(K, D, (640, 480), bands=(0.0, 1.5))
for k in range(8):
    one.step(frame([], k, glare=True), PAN, TILT)
b, _, _ = one.step(frame([], 9, glare=True), PAN, TILT)
check("6c (비교) 기준 하나면 먼 바닥이 띠로 잡힘", len(b) >= 1, [(round(x["near"], 2), x["area"]) for x in b])

# 7. 로봇 몸 자리 (11-18 실물: 고개 −30 에서 자기 앞부분을 후보로): 몸 둘레 + 5 cm 안 물건은 안 봄, 그 밖은 봄
PAN, TILT = 0.0, -30.0
fd = floor_detect.FloorDetect(K, D, (640, 480))
for k in range(8):
    fd.step(frame([], k), PAN, TILT)
b_in, _, _ = fd.step(frame([(0.10, 0.17, -0.06, 0.06)], 9), PAN, TILT)
b_out, _, _ = fd.step(frame([(0.30, 0.38, -0.06, 0.06)], 10), PAN, TILT)
check("7 몸 자리 안은 안 봄 / 밖 0.30 m 는 봄", not [x for x in b_in if x["near"] < 0.2] and len(b_out) == 1 and abs(b_out[0]["near"] - 0.30) < 0.03,
      ([round(x["near"], 2) for x in b_in], [round(x["near"], 3) for x in b_out]))

print(f"\n{sum(results)}/{len(results)} 통과")
sys.exit(0 if all(results) else 1)
