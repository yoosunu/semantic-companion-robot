#!/usr/bin/env python3
"""hazard_sim — 센서 사각 바닥 위험물 판단 (nodes/navigation/hazards.py) 시험 (11-19). ROS 없음.
   python3 tests/hazard_sim.py
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "nodes", "navigation"))
import hazards as hz  # noqa: E402

results = []


def check(name, ok, info=""):
    results.append(bool(ok))
    print(("통과 " if ok else "실패 ") + name + (f"  ({info})" if info != "" else ""))


def rect(cx, cy, w, h, n=6):
    xs, ys = np.linspace(cx - w / 2, cx + w / 2, n), np.linspace(cy - h / 2, cy + h / 2, n)
    return np.array([(x, cy - h / 2) for x in xs] + [(x, cy + h / 2) for x in xs] + [(cx - w / 2, y) for y in ys] + [(cx + w / 2, y) for y in ys])


wall = np.array([(2.0, y) for y in np.linspace(-1, 1, 60)])          # 멀리 벽 (LiDAR 점)
H = hz.HazardMap()

# 1. 발닦개 57×38 cm, 그 자리에 LiDAR 점 없음 → 표시
mat = hz.shape_for("doormat", rect(1.0, 0.2, 0.57, 0.38), [])
r1 = H.judge("doormat", 0.84, mat, wall)
check("1 발닦개 + LiDAR 0점 → 표시", r1[0] == "mark" and abs(hz.area(mat) - 0.57 * 0.38) < 0.01, (r1, round(hz.area(mat), 3)))

# 2. 의자에 걸린 수건: 이름은 바닥형인데 그 자리에 의자 다리 LiDAR 점 → 이름표만
legs = np.array([(0.8, -0.5), (0.81, -0.5), (1.2, -0.5), (1.21, -0.5), (0.8, -0.1), (1.2, -0.1)])
towel = hz.shape_for("towel", rect(1.0, -0.3, 0.45, 0.45), [])
r2 = H.judge("towel", 0.7, towel, np.vstack([wall, legs]))
check("2 걸린 수건 (의자 다리 점) → 이름표만", r2[0] == "skip" and "센서가 봄" in r2[1], r2)

# 3. 바닥형 아님 / 점수 낮음
check("3 chair → 안 찍음, doormat 0.40 → 안 찍음",
      H.judge("chair", 0.9, mat, wall)[0] == "skip" and H.judge("doormat", 0.40, mat, wall)[0] == "skip")

# 4. 신발: 아래 발자국 한 줄 → 6 cm 두께 띠
shoe = hz.shape_for("shoe", [], np.array([(0.7, 0.0), (0.7, 0.05), (0.71, 0.1)]))
r4 = H.judge("shoe", 0.6, shoe, wall)
check("4 신발 발자국 줄 → 띠로 표시", r4[0] == "mark" and 0.004 < hz.area(shoe) < 0.03, (r4, round(hz.area(shoe), 4)))

# 5. 떠 있는 것의 엉뚱한 발자국 (넓이 2 m²) → 안 찍음
big = hz.shape_for("towel", rect(2.5, 0, 1.6, 1.3), [])
check("5 넓이 2 m² → 안 찍음", H.judge("towel", 0.9, big, wall)[0] == "skip")

# 6. 같은 발닦개를 조금 옮겨 다시 봄 → 합침 (1 개)
far = hz.shape_for("doormat", rect(1.05, 0.22, 0.69, 0.59), [])     # 멀리서 본 것 (크게 나옴, 11-19 실측)
a = H.add("doormat", 0.84, far, 1.0, dist=1.2)
b = H.add("rug", 0.64, mat, 2.0, dist=0.7)                            # 가까이서 다시 봄 → 모양 바꿈, 이름은 점수 높은 것
c6 = H.add("doormat", 0.70, far, 3.0, dist=1.3)                       # 다시 멀리서 → 모양 유지
check("6 같은 자리 → 합침, 모양 = 가까이서 본 것, 이름 = 점수 높은 것", a == "new" and b == "replaced" and c6 == "kept" and len(H.items) == 1
      and H.items[0]["n"] == 3 and H.items[0]["name"] == "doormat" and abs(hz.area(H.items[0]["poly"]) - hz.area(mat)) < 1e-6,
      (a, b, c6, round(hz.area(H.items[0]["poly"]), 3)))

# 7. 다른 자리 → 따로
c = H.add("shoe", 0.6, shoe, 3.0)
check("7 다른 자리 → 새로 (2 개)", c == "new" and len(H.items) == 2)

# 8. costmap 점: 다각형 안을 3 cm 격자로 채움, 전부 안쪽
pts = H.cloud_points()
ok = all(hz.inside(hz.grow(it["poly"], 0.005), pts).any() for it in H.items)
inside_any = np.zeros(len(pts), bool)
for it in H.items:
    inside_any |= hz.inside(hz.grow(it["poly"], 0.005), pts)
check("8 costmap 점: 발닦개 격자 ~240 점, 전부 표시 안", ok and inside_any.all() and 200 < len(pts) < 400, len(pts))

# 9. 지우기
check("9 지우기 → 0 개", H.clear() == 2 and len(H.items) == 0 and len(H.cloud_points()) == 0)

# 10~12. 화면 끝에 잘린 조각 (11-20 실물: 같은 발닦개를 왼쪽 끝·오른쪽 끝에서 봄 → 삼각형 + 조각, 중심 36 cm, 따로 찍힘)
G = hz.HazardMap()
left = hz.hull(np.array([(0.55, 0.05), (0.85, 0.05), (0.85, 0.30), (0.55, 0.30), (0.70, 0.40)]))   # 왼쪽 조각
right = hz.hull(np.array([(0.60, -0.10), (0.95, -0.10), (0.95, 0.02), (0.60, 0.0)]))               # 오른쪽 조각 (8 cm 틈)
r10 = [G.add("doormat", 0.80, left, 1.0, 0.74, full=False), G.add("doormat", 0.83, right, 2.0, 0.87, full=False)]
check("10 잘린 두 조각 (8 cm 틈) → 한 물건으로 합침", r10 == ["new", "joined"] and len(G.items) == 1
      and hz.area(G.items[0]["poly"]) > hz.area(left) + hz.area(right) - 0.01, (r10, round(hz.area(G.items[0]["poly"]), 3)))
whole = hz.shape_for("rug", rect(0.75, 0.10, 0.57, 0.38), [])
r11 = G.add("rug", 0.67, whole, 3.0, 0.64, full=True)
check("11 통째로 본 모양 (rug) 이 오면 → 그걸로 바꿈, 이름은 점수 높은 doormat 유지", r11 == "replaced" and G.items[0]["full"]
      and abs(hz.area(G.items[0]["poly"]) - hz.area(whole)) < 1e-6 and G.items[0]["name"] == "doormat", r11)
r12 = G.add("doormat", 0.85, left, 4.0, 0.5, full=False)
check("12 통째가 있는데 잘린 조각 → 모양 그대로 (확인만)", r12 == "kept" and abs(hz.area(G.items[0]["poly"]) - hz.area(whole)) < 1e-6
      and len(G.items) == 1, r12)
check("13 rug 도 바닥형·납작 (발닦개와 같게)", G.judge("rug", 0.67, whole, wall)[0] == "mark")

# 14~17. 2단계: 떠 있나 (바닥 감지 기록). 카메라 높이 H 0.24, 물체 실제 앞 끝 X = 1.0 m
def seen(h, rxs, noise=0.0, seed=0):
    rng = np.random.default_rng(seed)
    k = 0.24 / (0.24 - h)
    return [(r, 0.0, r + k * (1.0 - r) + rng.normal(0, noise), rng.normal(0, noise)) for r in rxs]
v0 = hz.floating(seen(0.0, [0.0, 0.2, 0.45], noise=0.015))
v5 = hz.floating(seen(0.05, [0.0, 0.2, 0.45]))
v2 = hz.floating(seen(0.02, [0.0, 0.2, 0.45]))
vr = hz.floating([(0.0, 0.0, 1.0, 0.0), (0.01, 0.0, 1.0, 0.02)])
check("14 바닥 물체 (잡음 1.5 cm) → 바닥", v0[0] is False, tuple(round(x, 3) for x in v0[1:]))
check("15 5 cm 떠 있음, 45 cm 다가감 → 떠 있음 (끌려옴 ~12 cm)", v5[0] is True and 0.10 < v5[1] < 0.14, tuple(round(x, 3) for x in v5[1:]))
check("16 2 cm 떠 있음 → 바닥으로 침 (오차 10 % 미만)", v2[0] is False, tuple(round(x, 3) for x in v2[1:]))
check("17 거의 안 움직임 (제자리) → 판단 안 함", vr[0] is None)
Q = hz.HazardMap()
Q.add("towel", 0.7, hz.shape_for("towel", rect(1.3, 0.0, 0.4, 0.3), []), 0.0, 1.3)       # 떠 있는 것의 멀리 찍힌 발자국
ev = []
for r, (rx, ry, px, py) in zip(range(3), seen(0.05, [0.0, 0.25, 0.5])):
    ev += Q.observe(rx, ry, px, py)
check("18 표시 + 다가가며 본 덩어리 → 떠 있음 → 표시 뺌", [e[0] for e in ev] == ["floating"] and len(Q.items) == 0, [e[0] for e in ev])

check("19 기록 2 개로는 판정 안 함 (3 개부터)", hz.floating(seen(0.05, [0.0, 0.5]))[0] is None)

# 20~21. 11-20 02:09 실물 기록 그대로 (발닦개, 표시 중심 (+1.01, −0.01) 넓이 0.24 → 앞 가장자리 ~0.81).
#   장면별 덩어리 (가장 가까운 점 x, y). 로봇 0.12 에선 안쪽 조각 (0.99) 만, 이후엔 앞 끝 0.84 가 그대로 → 예전 판정은 "15 cm 끌려옴"
real = [(0.12, [(0.99, 0.06), (0.99, -0.02)]), (0.17, [(0.84, -0.04), (0.95, 0.05), (1.05, -0.19)]),
        (0.23, [(0.84, -0.07), (0.92, 0.11)]), (0.29, [(0.85, -0.04)])]
R = hz.HazardMap()
R.add("doormat", 0.77, hz.shape_for("doormat", rect(1.01, -0.01, 0.40, 0.60), []), 0.0, 1.0)
ev = []
for rx, pts in real:
    ev += R.observe_frame(rx, 0.0, pts, remove=False)
check("20 실물 기록 (조각 섞임) → 떠 있음 아님 (앞 끝 점만, 안쪽 조각 버림)", not ev and len(R.items) == 1
      and [round(o[2], 2) for o in R.items[0]["obs"]] == [0.84, 0.84, 0.85], [round(o[2], 2) for o in R.items[0]["obs"]])
for rx in (0.36, 0.43, 0.50):                                        # 더 다가감 (앞 끝 그대로) → 바닥 확인
    ev += R.observe_frame(rx, 0.0, [(0.845, -0.05), (0.97, 0.08)], remove=False)
check("21 더 다가가면 → 바닥 확인", [e[0] for e in ev] == ["ground"], [e[0] for e in ev])

Q2 = hz.HazardMap()                                                  # 18 과 같은 떠 있는 수건을 장면 단위로 (조각 하나 더 섞어서)
Q2.add("towel", 0.7, hz.shape_for("towel", rect(1.45, 0.0, 0.4, 0.3), []), 0.0, 1.3)
ev2 = []
for rx, ry, px, py in seen(0.05, [0.0, 0.2, 0.35, 0.5]):
    ev2 += Q2.observe_frame(rx, ry, [(px, py), (px + 0.15, 0.05)], remove=True)
check("22 떠 있는 것 (장면 단위, 조각 섞임) → 떠 있음", [e[0] for e in ev2] == ["floating"] and not Q2.items, [e[0] for e in ev2])

# 23~29. 11-25 DriftWatch: SLAM 이 map↔odom 을 고치면 지역 costmap (odom 좌표) 의 옛 표시가 밀림 (run_1125d 침대 두 겹)
import math  # noqa: E402
D = hz.DriftWatch(dist=0.05, window=1.5, gap=5.0)
check("23 처음 한 번은 기준만 잡음", D.update((0.0, 0.0, 0.0), (1.0, 0.0), 0.0) is None)
check("24 잔떨림 (2 cm, 0.5°) → 안 지움", D.update((0.02, 0.0, math.radians(0.5)), (1.0, 0.0), 10.0) is None,
      round(D.drift((0.02, 0.0, math.radians(0.5)), (1.0, 0.0)), 3))
d = D.update((0.06, 0.0, 0.0), (1.0, 0.0), 20.0)
check("25 6 cm 보정 → 지움", d is not None and abs(d - 0.06) < 1e-6, d)
check("26 바로 또 보정 (5 초 안) → 기다림", D.update((0.0, 0.0, 0.0), (1.0, 0.0), 22.0) is None)
d = D.update((0.0, 0.0, 0.0), (1.0, 0.0), 26.0)
check("27 5 초 지나면 → 지움 (기준은 지운 때 자세)", d is not None and abs(d - 0.06) < 1e-6, d)
E = hz.DriftWatch()
E.update((0.0, 0.0, 0.0), (0.0, 0.0), 0.0)
out = [E.update((0.02 * k, 0.0, 0.0), (0.0, 0.0), 10.0 * k) for k in (1, 2, 3)]
check("28 조금씩 쌓인 어긋남 (2 cm × 3) → 세 번째에 지움", out[0] is None and out[1] is None and out[2] is not None, out)
F1, F2 = hz.DriftWatch(), hz.DriftWatch()
F1.update((0.0, 0.0, 0.0), (0.0, 0.0), 0.0)
F2.update((0.0, 0.0, 0.0), (3.0, 0.0), 0.0)
r1 = F1.update((0.0, 0.0, math.radians(1.0)), (0.0, 0.0), 10.0)
r2 = F2.update((0.0, 0.0, math.radians(1.0)), (3.0, 0.0), 10.0)
check("29 같은 1° 회전: 로봇이 odom 원점 → 안 지움, 3 m 떨어짐 → 지움 (멀수록 많이 밀림)", r1 is None and r2 is not None,
      f"{F1.drift((0, 0, math.radians(1.0)), (0, 0)):.3f} / {r2}")

print(f"\n{sum(results)}/{len(results)} 통과")
