#!/usr/bin/env python3
"""glance (안경 쓰기 판단) 시뮬레이션. python3 tests/glance_sim.py"""
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "nodes", "perception"))
import glance as gl  # noqa: E402
import ground  # noqa: E402

results = []


def check(name, ok, info=""):
    results.append(bool(ok))
    print(f"{'OK ' if ok else 'XX '} {name} {info}")


# 1. 좌표 바꾸기: 왕복 / 90° 돌면 앞 물건이 오른쪽으로 (로봇 좌표), 세상 좌표는 그대로
pose = (1.0, 0.5, math.radians(30))
wx, wy = gl.to_world(pose, 0.5, 0.1)
bx, by = gl.to_robot(pose, wx, wy)
p0, p1 = (0, 0, 0), (0, 0, math.radians(90))
w0 = gl.to_world(p0, 0.5, 0.0)
r1 = gl.to_robot(p1, *w0)
check("1 좌표: 왕복 / 왼쪽으로 90° 돌면 앞 0.5 m → 오른쪽 0.5 m", abs(bx - 0.5) < 1e-9 and abs(by - 0.1) < 1e-9
      and abs(r1[0]) < 1e-9 and abs(r1[1] + 0.5) < 1e-9, (round(r1[0], 3), round(r1[1], 3)))

# 2. 로봇이 앞으로 가면서 같은 물건 (세상 1.5, 0.1) 을 봄 → 2 번째에 후보, 한 번 깜빡인 건 후보 아님
c = gl.Candidates()
out = []
for k in range(4):
    pose = (1.0 + 0.05 * k, 0.0, 0.0)
    x, y = gl.to_robot(pose, 1.5, 0.1)
    blobs = [(x, y, 300)] + ([(0.6, -0.3, 80)] if k == 0 else [])           # 깜빡: 처음 한 번만
    out.append(c.step(k * 0.5, pose, blobs))
check("2 움직이며 같은 자리 2 번 → 후보 (깜빡인 건 아님)", out[0] is None and out[1] and abs(out[1]["wx"] - 1.5) < 0.02
      and abs(out[1]["wy"] - 0.1) < 0.02 and all(o is None or abs(o["wy"] - 0.1) < 0.02 for o in out), out[1])

# 3. 본 곳은 30 분 동안 다시 안 봄, 그 뒤엔 다시
c.remember(out[1]["wx"], out[1]["wy"], 1.0, "doormat", 0.8, pose=(1.0, 0, 0))
again = c.step(2.0, (1.2, 0, 0), [gl.to_robot((1.2, 0, 0), 1.5, 0.1) + (300,)])
later = [c.step(2000 + k * 0.5, (1.2, 0, 0), [gl.to_robot((1.2, 0, 0), 1.5, 0.1) + (300,)]) for k in range(2)]
check("3 본 곳은 계속 기억 (오래 지나도 안 봄)", again is None and later[-1] is None)

# 3b. 쓰레기통 (11-18 실물): 고개 각도마다 바닥 닿은 점이 25 cm 흔들려도 같은 것 — 한 번 보면 다시 안 봄
import random as _r
rng = _r.Random(2)
c = gl.Candidates()
asked = 0
for k in range(30):
    jx, jy = rng.uniform(-0.12, 0.12), rng.uniform(-0.12, 0.12)
    cand = c.step(k * 0.5, (0, 0, 0), [(0.75 + jx, -0.35 + jy, 400)])
    if cand:
        asked += 1
        c.remember(cand["wx"], cand["wy"], k * 0.5, "trash can", 0.5, pose=(0, 0, 0))
check("3b 흔들리는 물건 (±12 cm) 은 한 번만 봄", asked == 1, f"{asked} 번")

# 3c. 사라짐: 보여야 하는데 2 번 안 잡히면 지움 / 고개 돌려 화면 밖이면 안 지움
c = gl.Candidates()
c.step(0.0, (0, 0, 0), [(0.6, 0.0, 300)])
c.step(0.5, (0, 0, 0), [], visible=lambda x, y: False)
c.step(1.0, (0, 0, 0), [], visible=lambda x, y: False)
kept = len(c.tracks)
c.step(1.5, (0, 0, 0), [], visible=lambda x, y: True)
c.step(2.0, (0, 0, 0), [], visible=lambda x, y: True)
check("3c 화면 밖은 안 지움, 보여야 하는데 2 번 없으면 지움", kept == 1 and len(c.tracks) == 0, (kept, len(c.tracks)))

# 4. 멀면 (1.2 m 넘게) 안 봄
c = gl.Candidates()
far = [c.step(k * 0.5, (0, 0, 0), [(1.4, 0.0, 300)]) for k in range(3)]
check("4 1.4 m 는 안 봄", all(f is None for f in far))

# 5. head_for: 그 바닥 점이 화면 가운데 근처 (가로 ±15 px, 세로 ±40 px — 틸트 −30 한계 밖은 제외)
K = np.array([[319.9, 0, 321.0], [0, 320.1, 224.4], [0, 0, 1]])
D = np.array([-0.0229, -0.0288, 0.0251, -0.0102]).reshape(4, 1)
errs = []
for x, y in [(0.5, 0.0), (0.8, 0.3), (0.6, -0.4), (1.0, 0.0), (0.4, 0.2)]:
    pan, tilt = gl.head_for(x, y)
    px = ground.floor_to_pixels([[x, y]], K, D, pan, tilt)[0]
    errs.append((round(pan), round(tilt), round(px[0] - K[0, 2]), round(px[1] - K[1, 2])))
check("5 head_for: 점이 화면 가운데", all(abs(e[2]) <= 15 and (abs(e[3]) <= 40 or e[1] == -30) for e in errs), errs)

# 6. 3D → 픽셀 (11-19): 바닥 점은 예전 floor_to_pixels 와 같고, 바닥 → 픽셀 → 바닥 왕복
fl = np.array([[0.5, 0.1], [0.9, -0.3], [0.4, 0.25]])
px = ground.floor_to_pixels(fl, K, D, -20, -15)
back = ground.pixels_to_floor(px, K, D, -20, -15)
p3 = ground.points_to_pixels(np.column_stack([fl, np.zeros(3)]), K, D, -20, -15)
check("6 바닥 ↔ 픽셀 왕복 (1 mm), 3D 함수와 같음", np.abs(back - fl).max() < 1e-3 and np.abs(p3 - px).max() < 1e-9,
      round(float(np.abs(back - fl).max() * 1000), 3))

# 7. 자기 몸 자리 (11-19 사진 #3: 고개 −12, −30 → LiDAR 가 화면 아래 가운데, 윗면 앞 끝 y≈350)
b = ground.body_mask(K, D, -12, -30, 640, 480)
b0 = ground.body_mask(K, D, 0, 0, 640, 480)
col = np.nonzero(b[:, 380])[0]
check("7 몸 자리: −30 에서 LiDAR 아래 가운데 (위 끝 y 320~380), 위 절반 없음 / 고개 0 이면 거의 없음",
      len(col) and 320 <= col.min() <= 380 and not b[:240].any() and b0.mean() < 0.03,
      (int(col.min()) if len(col) else None, round(float(b0.mean()), 3)))

# 8. 흐림: 같은 장면 안에서 비교 (11-19 흰 벽: 선명도가 원래 1200 대 → 예전엔 '평소 2471' 의 절반 못 넘어 전부 버림)
wall = gl.pick_sharp([1250, 1266, 1240], 2, 0.6, 3)
shake = [gl.pick_sharp([300, 1300], 2, 0.6, 3), gl.pick_sharp([300, 1300, 1280], 2, 0.6, 3)]
def stream(shs):                                                         # 한 장씩 들어올 때 처음 정해지는 순간
    return next((r for k in range(1, len(shs) + 1) if (r := gl.pick_sharp(shs[:k], 2, 0.6, 3)) is not None), None)
dull = stream([400, 2000, 450, 420, 430])
check("8 흐림: 밋밋한 장면도 통과 / 흔들린 첫 장은 기다렸다 버림 / 하나만 튀면 5 장째에 가장 선명한 2 장",
      wall == [0, 1] and shake == [None, [1, 2]] and dull == [1, 2], (wall, shake, dull))

# 9. 같은 자리 2 번 포기 → '?' 로 기억 → 다시 후보 안 됨 (11-19: 같은 곳 7 번 요청)
c = gl.Candidates()
first = c.fail(0.5, 0.7, 1.0, pose=(0, 0, 0))
second = c.fail(0.52, 0.71, 6.0, pose=(0, 0, 0))
again = [c.step(10 + k * 0.5, (0, 0, 0), [(0.5, 0.7, 300)]) for k in range(3)]
other = c.fail(-0.5, 0.2, 7.0, pose=(0, 0, 0))
check("9 포기 2 번 → '?' 기억, 다시 안 봄 (다른 자리는 따로 셈)", not first and second and all(a is None for a in again) and not other
      and c.known(0.5, 0.7, 11, pose=(0, 0, 0))[3] == "?", (first, second))

# 10. 건너뛴 (연속 아닌) 배열도 같은 결과 (11-20 버그: 윤곽 c[::k] 를 OpenCV 가 잘못 읽음)
pts = np.column_stack([np.linspace(150, 420, 60), np.linspace(170, 230, 60)])
a10 = ground.pixels_to_floor(pts[::3], K, D, 8, -26)
b10 = ground.pixels_to_floor(np.ascontiguousarray(pts[::3]), K, D, 8, -26)
check("10 건너뛴 배열 = 연속 배열 (바닥 좌표 같음)", np.allclose(a10, b10, equal_nan=True), round(float(np.nanmax(np.abs(a10 - b10))), 4))

# 11. 따라보기 판단 (11-20)
fv = gl.focus_verdict
c11 = [fv({"chair": [0.81]}, 1, True, None)[:2], fv({"chair": [0.81, 0.77]}, 2, True, None)[:2],
       fv({}, 2, False, None)[:2], fv({"towel": [0.3, 0.4]}, 2, True, None)[:2],
       fv({"doormat": [0.8, 0.7]}, 2, False, None)[:2], fv({"doormat": [0.8, 0.7]}, 2, True, {"ground": None})[:2],
       fv({"doormat": [0.8, 0.7]}, 3, True, {"ground": True})[:2], fv({"rug": [0.7, 0.6], "doormat": [0.8, 0.75]}, 3, True, None)[:2],
       fv({"shelf": [0.23]}, 3, False, None)[:2], fv({"shelf": [0.51, 0.48], "laundry basket": [0.50]}, 3, False, None)[:2]]
check("11 따라보기: 의자 1번 = 더 봄 / 2번 = 끝 / 빈 바닥 = 끝 / 애매 2번 = 더 봄 / 발닦개 잘림 = 더 봄 / 통째면 끝 (바닥 확인 안 기다림) "
      "/ 3번 봐도 0.5 없음 = ? 끝 / 이름 갈려도 0.5 두 번 = 끝",
      c11 == [(False, "?"), (True, "chair"), (True, "-"), (False, "?"), (False, "doormat"), (True, "doormat"),
              (True, "doormat"), (True, "doormat"), (True, "?"), (True, "shelf")], c11)

on4 = lambda n: 0.4 if n in ("trash can", "doormat") else 0.5
c12 = [fv({"trash can": [0.445, 0.451, 0.491]}, 3, True, None, on_for=on4)[:2], fv({"trash can": [0.445, 0.451, 0.491]}, 3, True, None)[:2],
       fv({"chair": [0.45, 0.48, 0.44]}, 3, True, None, on_for=on4)[:2]]
check("12 YOLOE 단어 문턱 0.4: 휴지통 0.45·0.45·0.49 → 확신 (0.5 면 ?) / COCO 의자 0.45 → ? (0.5 그대로)",
      c12 == [(True, "trash can"), (True, "?"), (True, "?")], c12)

print(f"\n{sum(results)}/{len(results)} 통과")
sys.exit(0 if all(results) else 1)
