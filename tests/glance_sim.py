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

# 3. 본 곳은 5 분 동안 다시 안 봄, 그 뒤엔 다시
c.remember(out[1]["wx"], out[1]["wy"], 1.0, "doormat", 0.8)
again = c.step(2.0, (1.2, 0, 0), [gl.to_robot((1.2, 0, 0), 1.5, 0.1) + (300,)])
later = [c.step(400 + k * 0.5, (1.2, 0, 0), [gl.to_robot((1.2, 0, 0), 1.5, 0.1) + (300,)]) for k in range(2)]
check("3 본 곳 기억 5 분", again is None and later[-1] is not None)

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

print(f"\n{sum(results)}/{len(results)} 통과")
sys.exit(0 if all(results) else 1)
