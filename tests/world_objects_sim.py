#!/usr/bin/env python3
"""world_objects_sim — 네리가 '알고 있는' 객체 추적 (nodes/navigation/world_objects.py) 시험 (11-27). ROS 없음.
   python3 tests/world_objects_sim.py          가짜 장면
   python3 tests/world_objects_sim.py --real   + 11-27 실물 녹화 logs/scan/chair_push.jsonl (의자 밀기·가리기·굴리기, 45 s)
"""
import json
import math
import os
import sys

import numpy as np

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
sys.path.insert(0, os.path.join(ROOT, "nodes", "navigation"))
import world_objects as wo  # noqa: E402

results = []


def check(name, ok, info=""):
    results.append(bool(ok))
    print(("통과 " if ok else "실패 ") + name + (f"  ({info})" if info != "" else ""))


SENSOR = np.array([0.0, 0.0])


BOX = ((3.0, -3.0, 3.0, 3.0), (-3.0, 3.0, 3.0, 3.0), (-3.0, -3.0, -3.0, 3.0), (-3.0, -3.0, 3.0, -3.0))   # 방 6 × 6 m


def scan(circles, walls=BOX, drop=False, sensor=None):
    """센서에서 0.5° 광선 720 개 → 스캔 순서 점. circles = [(x, y, 반지름)], walls = 선분"""
    S = SENSOR if sensor is None else np.asarray(sensor, float)
    pts = []
    for a in np.arange(-math.pi, math.pi, math.radians(0.5)):
        d = np.array([math.cos(a), math.sin(a)])
        best = math.inf
        for cx, cy, r in circles:
            if drop and (cx, cy) == drop:
                continue
            oc = np.array([cx, cy]) - S
            b = oc @ d
            h = oc @ oc - b * b
            if b > 0 and h <= r * r:
                best = min(best, b - math.sqrt(r * r - h))
        for x0, y0, x1, y1 in walls:
            p, q = np.array([x0, y0]), np.array([x1, y1])
            m = np.array([[d[0], p[0] - q[0]], [d[1], p[1] - q[1]]])
            if abs(np.linalg.det(m)) < 1e-9:
                continue
            tt, s = np.linalg.solve(m, p - S)
            if tt > 0 and 0 <= s <= 1:
                best = min(best, tt)
        if math.isfinite(best):
            pts.append(S + best * d)
    return np.array(pts)


COL = (1.0, 0.0, 0.02)                                  # 의자 기둥 지름 4 cm

# 1. 가만히 + 가끔 한 장 놓침 → 계속 있음
W = wo.WorldObjects()
o, _ = W.adopt("chair", (1.0, 0.0), t=0.0)
ev = []
for k in range(30):
    ev += W.step(scan([COL], drop=(1.0, 0.0) if k % 7 == 3 else False), SENSOR, k * 0.1)
check("1 가만히 + 7 장마다 한 장 놓침 → 사건 없음, 있음", not ev and o.state == wo.PRESENT, ([e[0] for e in ev], o.state))

# 2. 사람 다리가 앞을 가림 → 가려짐 (없음 아님), 비키면 다시 찾음
ev = []
for k in range(30, 60):
    ev += W.step(scan([COL, (0.5, 0.0, 0.06)]), SENSOR, k * 0.1)
check("2 앞에 다리 3 s → 가려짐, 없음 의심 아님", o.state == wo.HIDDEN and "missing" not in [e[0] for e in ev], [e[0] for e in ev])
ev = W.step(scan([COL]), SENSOR, 6.1)
check("3 비키면 → 다시 찾음 (found)", [e[0] for e in ev] == ["found"] and o.state == wo.PRESENT, [e[0] for e in ev])

# 4. 천천히 밀기 50 cm (장마다 2 cm) → moving → 멈추고 1 s → moved 50 cm
ev, t = [], 6.2
for k in range(26):
    ev += W.step(scan([(1.0, 0.02 * k, 0.02)]), SENSOR, t)
    t += 0.1
for k in range(15):
    ev += W.step(scan([(1.0, 0.50, 0.02)]), SENSOR, t)
    t += 0.1
kinds = [e[0] for e in ev]
mv = [e[2] for e in ev if e[0] == "moved"]
check("4 천천히 50 cm 밀기 → moving 다음 moved (거리 ~0.50), 같은 의자", kinds == ["moving", "moved"] and abs(mv[0]["dist"] - 0.50) < 0.02
      and len(W.objs) == 1, (kinds, mv))

# 5. 빠르게 굴리기 (장마다 5 cm) → 끊기지 않고 따라감
ev = []
for k in range(1, 21):
    ev += W.step(scan([(1.0 - 0.05 * k * 0.0, 0.50 + 0.05 * k, 0.02)]), SENSOR, t)
    t += 0.1
check("5 장마다 5 cm (빠르게 굴리기) → 따라감 (1.0, 1.5)", np.hypot(*(o.anchor - [1.0, 1.5])) < 0.02 and "missing" not in [e[0] for e in ev],
      (o.anchor.round(2).tolist(), [e[0] for e in ev]))

# 6. 한 번에 순간이동 (가려진 사이 옮김) → 못 따라감 → 시선이 빈자리를 지나가면 1 s 뒤 없음 의심
ev = []
for k in range(15):
    ev += W.step(scan([(2.0, -1.0, 0.02)]), SENSOR, t)
    t += 0.1
check("6 순간이동 → 따라가지 않음, 1 s 뒤 missing (새 자리는 새 객체 몫)",
      "missing" in [e[0] for e in ev] and np.hypot(*(o.anchor - [1.0, 1.5])) < 0.02, ([e[0] for e in ev], o.anchor.round(2).tolist()))

# 7. 의자 둘 30 cm 옆 — 하나만 밀면 다른 하나는 그대로 (한 덩어리는 한 닻에만)
W2 = wo.WorldObjects()
a, _ = W2.adopt("chair", (1.0, 0.0))
b, _ = W2.adopt("chair", (1.0, 0.30))
check("7a 30 cm 떨어진 의자 둘 → 객체 둘", len(W2.objs) == 2)
t = 0.0
for k in range(10):
    W2.step(scan([(1.0, -0.03 * k, 0.02), (1.0, 0.30, 0.02)]), SENSOR, t)
    t += 0.1
check("7b 하나만 밀면 → 그 의자만 움직이고 다른 하나는 제자리 (LiDAR 는 기둥 앞면을 봐서 ~2 cm 앞)",
      np.hypot(*(a.anchor - [1.0, -0.27])) < 0.03 and np.hypot(*(b.anchor - [1.0, 0.30])) < 0.03,
      (a.anchor.round(2).tolist(), b.anchor.round(2).tolist()))

# 8. 모양은 닻을 따라감
W3 = wo.WorldObjects()
c, _ = W3.adopt("chair", (1.0, 0.0), shape=[[0.6, -0.4], [1.4, -0.4], [1.4, 0.4], [0.6, 0.4]])
for k in range(10):
    W3.step(scan([(1.0, 0.05 * k, 0.02)]), SENSOR, k * 0.1)
check("8 닻이 45 cm 가면 모양 꼭짓점도 45 cm", np.allclose(c.corners()[0], [0.6, 0.05], atol=0.03), c.corners()[0].round(2).tolist())


# 9~14. 11-27 v2: 다리 여러 개 (강체). 닻 = LiDAR 로 확인한 다리만
def legs(cx, cy, th, half=0.2, r=0.015):
    """4 다리 의자 (40 cm 정사각형) — 가운데 (cx, cy), 방향 th → 다리 원 4 개"""
    R = wo.rot(th)
    return [tuple((R @ np.array([dx, dy]) + [cx, cy]).tolist()) + (r,) for dx in (-half, half) for dy in (-half, half)]


W4 = wo.WorldObjects()
c4, _ = W4.adopt("chair", [(x, y) for x, y, _ in legs(1.2, 0.0, 0.0)], t=0.0)
ev, t = [], 0.0
for k in range(1, 31):                                   # 장마다 1 cm 옆으로 + 1° 돌림 → 30 cm, 30°
    ev += W4.step(scan(legs(1.2, 0.01 * k, math.radians(k))), SENSOR, t)
    t += 0.1
for k in range(15):
    ev += W4.step(scan(legs(1.2, 0.30, math.radians(30))), SENSOR, t)
    t += 0.1
mv = [e[2] for e in ev if e[0] == "moved"]
check("9 4 다리 의자 밀며 돌림 (30 cm, 30°) → moved 하나, 거리 ~0.30, 회전 ~30°",
      len(mv) == 1 and abs(mv[0]["dist"] - 0.30) < 0.03 and abs(mv[0]["turn"] - 30) < 3, mv)

S2 = np.array([0.0, 0.2])                                 # 센서가 오른쪽 다리 줄과 일직선 → 뒷다리가 앞다리에 가림
W5 = wo.WorldObjects()
c5, _ = W5.adopt("chair", [(x, y) for x, y, _ in legs(1.2, 0.0, 0.0)], t=0.0)
ev = []
for k in range(30):
    ev += W5.step(scan(legs(1.2, 0.0, 0.0), sensor=S2), S2, k * 0.1)
check("10 뒷다리가 앞다리에 가림 (네 개 중 셋만 보임) → 사건 없음, 있음", not ev and c5.state == wo.PRESENT and c5.last["hidden"] >= 1,
      ([e[0] for e in ev], c5.last))

ev = []
for k in range(30, 60):                                   # 사람 다리가 앞다리 둘을 가림
    ev += W5.step(scan(legs(1.2, 0.0, 0.0) + [(0.6, 0.1, 0.12)], sensor=S2), S2, k * 0.1)
check("11 사람이 다리 일부를 가림 → 남은 다리로 계속 있음 (없음 의심 아님)", "missing" not in [e[0] for e in ev] and c5.state == wo.PRESENT,
      ([e[0] for e in ev], c5.last))

BED = [(1.45, 0.65, 0.03), (1.47, -0.30, 0.03)]           # 11-25 실물 침대: LiDAR 로 확인한 다리 2 개 (간격 0.95)
DRAWER = ((1.0, 1.0, 1.6, 1.0),)                          # 서랍장 (일부 각도에서 다리를 가림)
Wb = wo.WorldObjects()
bed, _ = Wb.adopt("bed", [(x, y) for x, y, _ in BED], t=0.0)
ev, t = [], 0.0
for k in range(80):                                       # 로봇이 침대 앞을 왔다 갔다 (센서 자리가 계속 바뀜)
    sx, sy = 0.2 + 0.4 * math.sin(k * 0.08), -1.0 + 2.0 * k / 80
    ev += Wb.step(scan(BED, walls=BOX + DRAWER, sensor=(sx, sy)), (sx, sy), t)
    t += 0.1
check("12 침대 (닻 2 개) 앞을 로봇이 오가며 봄 → 거짓 moving·missing 없음", not [e for e in ev if e[0] in ("moving", "missing", "moved")]
      and np.hypot(*(bed.p - [1.46, 0.175])) < 0.03, ([e[0] for e in ev], bed.p.round(3).tolist()))

rng = np.random.default_rng(3)
W6 = wo.WorldObjects()
c6, _ = W6.adopt("chair", [(x, y) for x, y, _ in legs(1.2, 0.0, 0.0)], t=0.0)
ev = []
for k in range(60):                                       # odom 이 장마다 ±1.5 cm, ±1° 흔들림 (주행 중 오차 흉내)
    P = scan(legs(1.2, 0.0, 0.0))
    d, a = rng.normal(0, 0.015, 2), math.radians(rng.normal(0, 1.0))
    ev += W6.step(P @ wo.rot(a).T + d, SENSOR, k * 0.1)
check("13 odom 흔들림 (±1.5 cm, ±1°) → 거짓 moving 없음", not [e for e in ev if e[0] in ("moving", "moved", "missing")],
      ([e[0] for e in ev], c6.p.round(3).tolist()))

o1, new1 = W6.adopt("chair", [(1.01, 0.21)], t=7.0)
check("14 이미 있는 의자 다리 근처로 다시 맡기기 → 새로 안 만들고 같은 객체", not new1 and o1 is c6 and len(W6.objs) == 1)

# 15~16. 11-27 실물 첫 주행에서 빠르게 굴리자 놓침 → 속도 예측 + 움직이는 중 문턱 25 cm
W7 = wo.WorldObjects()
c7, _ = W7.adopt("chair", (1.0, 0.0), t=0.0)
ev, t, y = [], 0.0, 0.0
for k in range(40):                                       # 가만히 → 점점 빨라져 한 장에 15 cm (1.5 m/s) → 멈춤
    step = [0.0] * 5 + [0.03, 0.06, 0.10, 0.15, 0.15, 0.15, 0.15, 0.12, 0.08, 0.04] + [0.0] * 25
    y += step[k]
    ev += W7.step(scan([(1.0, y, 0.02)]), SENSOR, t)
    t += 0.1
mv = [e[2] for e in ev if e[0] == "moved"]
check("15 빠르게 굴리기 (한 장 최대 15 cm, 1.5 m/s) → 놓치지 않고 moved (~1.03 m), 없음 의심 없음",
      "missing" not in [e[0] for e in ev] and len(mv) == 1 and abs(mv[0]["dist"] - 1.03) < 0.05, ([e[0] for e in ev], mv))
W8 = wo.WorldObjects()
o8, n8 = W8.adopt("chair", (1.0, 0.0), key="F1")
for k in range(1, 21):
    W8.step(scan([(1.0, 0.03 * k, 0.02)]), SENSOR, k * 0.1)
o9, n9 = W8.adopt("chair", (1.0, 0.0), key="F1")       # 맡기는 쪽은 아직 옛 자리로 다시 보냄
check("16 같은 key 로 옛 자리를 다시 맡겨도 → 새 객체 안 만들고 LiDAR 가 따라간 자리 그대로 (유령 #2 방지)",
      not n9 and o9 is o8 and len(W8.objs) == 1 and abs(o8.p[1] - 0.6) < 0.03, (len(W8.objs), o8.p.round(2).tolist()))

# 17. 11-27 탐색 주행: odom 회전 드리프트 (150 s 에 24°) 로 가만있는 침대가 '옮겨짐 0.10~0.16 m' 반복
#     → 속도 조건 (최근 2 s 안에 10 cm). 장마다 무작위 흔들림 (13) 이 아니라 한쪽으로 쌓이는 드리프트
Wd = wo.WorldObjects()
bd, _ = Wd.adopt("bed", [(x, y) for x, y, _ in BED], t=0.0)
ev = []
for k in range(1500):                                     # 150 s 동안 odom 이 0.16°/s 로 돌아감 (총 24°) — 세상은 그대로
    a = math.radians(0.16 * k * 0.1)
    P = scan(BED, walls=BOX + DRAWER, sensor=(0.2, 0.0)) if k % 10 == 0 or k == 0 else P
    ev += Wd.step(P @ wo.rot(a).T, wo.rot(a) @ np.array([0.2, 0.0]), k * 0.1)
check("17 odom 이 150 s 에 24° 돌아가는 드리프트 → 가만있는 침대에 moving·moved·missing 없음",
      not [e for e in ev if e[0] in ("moving", "moved", "missing")], [e[0] for e in ev][:6])


def real(path):
    """11-27 실물: 로봇 정지, 의자 (기둥) 를 앞 0.97 m 에 → 0~10 s 가만히, 10~20 왼쪽으로 ~55 cm 밀기, 20~28 사람이 앞에 서기,
       28~35 왼쪽으로 빠르게 굴리기, 35~45 가만히. Jasper 확인: 처음부터 끝까지 한 의자 (책상 쪽 왼쪽으로)."""
    L = [json.loads(x) for x in open(path)]
    sensor = np.array([0.078, 0.0])                     # 로봇 정지 → 로봇 좌표 = odom 으로 씀. 라이다 180° 돌려 달림
    W = wo.WorldObjects()
    o, _ = W.adopt("chair", (0.97, 0.02), t=L[0]["t"])
    log, states = [], []
    for s in L:
        r = np.array(s["r"])
        a = s["amin"] + np.arange(len(r)) * s["ainc"]
        ok = np.isfinite(r) & (r > 0.15) & (r < 12)
        pts = np.column_stack([-(r * np.cos(a))[ok] + 0.078, -(r * np.sin(a))[ok]])
        for e in W.step(pts, sensor, s["t"]):
            log.append((round(s["t"], 1), e[0], e[2]))
        states.append((s["t"], o.state))
    print("--- 실물 chair_push: 사건")
    for x in log:
        print("  ", x)
    st = lambda t0, t1: {s for t, s in states if t0 <= t < t1}
    check("R1 끝까지 의자 하나, 마지막 자리 = 왼쪽 (0.78, 1.26) 5 cm 안", len(W.objs) == 1 and np.hypot(*(o.anchor - [0.78, 1.26])) < 0.05,
          o.anchor.round(2).tolist())
    check("R2 0~10 s 가만히: 있음만 (한 장 놓침에 흔들리지 않음)", st(0, 10) == {wo.PRESENT}, st(0, 10))
    check("R3 밀기 → moving, 멈춘 뒤 moved 0.4 m 넘게", any(k == "moved" and i["dist"] > 0.4 and 15 < t < 30 for t, k, i in log),
          [(t, k, i.get("dist")) for t, k, i in log if k == "moved"])
    check("R4 사람이 앞에 섬 → 가려짐, 없음 의심 없음 (28 s 전)", wo.HIDDEN in st(20, 33) and not any(k == "missing" and t < 33 for t, k, _ in log),
          st(20, 33))
    check("R5 빠르게 굴림 → 끊기지 않고 moved (왼쪽 끝까지)", any(k == "moved" and t > 33 and i["to"][1] > 1.1 for t, k, i in log),
          [(t, k, i.get("to")) for t, k, i in log if k == "moved"])


def real_drive(path):
    """11-27 탐색 뒤 주행 녹화 (150 s, Nav2 목표 + 제자리 90° 회전 여러 번). 침대·의자 아무도 안 건드림 (Jasper 확인).
       침대 다리 2 개 (map (-0.14, 0.65), (-0.12, -0.31)) 를 odom 으로 맡기고 그대로 흘림 → 사건 0 이어야 함"""
    L = [json.loads(x) for x in open(path)]
    ap = lambda T, P: np.column_stack([T[0] + P[:, 0] * math.cos(T[2]) - P[:, 1] * math.sin(T[2]),
                                       T[1] + P[:, 0] * math.sin(T[2]) + P[:, 1] * math.cos(T[2])])
    W, bed, ev, yaws = wo.WorldObjects(), None, [], []
    for s in L:
        if not s["odom_at"] or not s["map_odom"]:
            continue
        yaws.append(math.degrees(s["map_odom"][2]))
        r = np.array(s["r"])
        a = s["amin"] + np.arange(len(r)) * s["ainc"]
        ok = np.isfinite(r) & (r > 0.15) & (r < 12)
        Po = ap(s["odom_at"], np.column_stack([r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok])]))
        if bed is None:
            x, y, th = s["map_odom"]
            c, si = math.cos(th), math.sin(th)
            inv = (-(x * c + y * si), -(-x * si + y * c), -th)
            bed, _ = W.adopt("bed", ap(inv, np.array([[-0.14, 0.65], [-0.12, -0.31]])), t=s["t"])
            continue
        ev += [(round(s["t"], 1), e[0]) for e in W.step(Po, np.array(s["odom_at"][:2]), s["t"])]
    print(f"--- 실물 drive: map↔odom 방향 {min(yaws):.1f}° ~ {max(yaws):.1f}° (드리프트), 사건 {ev}")
    check("R6 주행 150 s (드리프트 24°), 가만있는 침대 → moving·moved·missing 없음", not [e for e in ev if e[1] in ("moving", "moved", "missing")], ev[:6])


if "--real" in sys.argv:
    for name, fn in (("chair_push.jsonl", real), ("drive.jsonl", real_drive)):
        p = os.path.join(ROOT, "logs", "scan", name)
        if os.path.exists(p):
            fn(p)
        else:
            print("실물 기록 없음:", p)
print(f"\n{sum(results)}/{len(results)} 통과")
