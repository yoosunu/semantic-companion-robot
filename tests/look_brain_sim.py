#!/usr/bin/env python3
"""look_brain 시뮬레이션 (ROS 없이). python3 tests/look_brain_sim.py
   가짜 /people 을 시간에 따라 넣고, 모드·동작이 규칙대로 나오는지 확인."""
import os
import random
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "scripts"))
import look_brain as lb  # noqa: E402

DT = 0.1
results = []


def check(name, ok, info=""):
    results.append(bool(ok))
    print(f"{'OK ' if ok else 'XX '} {name} {info}")


def person(pan=0.0, tilt=30.0, px=25.0, yaw=60.0, pid=1, unseen=0.0):
    return {"id": pid, "pan": pan, "tilt": tilt, "px": px, "yaw": yaw, "pitch": 0.0, "unseen": unseen}


def run(world, secs, b=None, motion=lambda t: False, t0=0.0):
    """world(t) → people 목록. 기록: [(t, 모드, 이유, 동작들)]"""
    b = b or lb.Brain(random.Random(1))
    log = []
    t = t0
    while t < t0 + secs:
        acts = b.tick(t, world(t), motion(t))
        log.append((round(t, 1), b.mode, b.reason, acts))
        t += DT
    return b, log


def modes(log):
    seq = []
    for t, m, r, _ in log:
        if not seq or seq[-1][1] != m:
            seq.append((t, m))
    return seq


def first(log, mode):
    return next((t for t, m, _, _ in log if m == mode), None)


# 1. 아무도 없음 → 두리번 → 1 분 뒤 꾸벅 → 3 분 뒤 잠 (얼굴 처리 끔, LED 끔)
b, log = run(lambda t: [], 260)
moves = sum(1 for *_, a in log for x in a if x[0] == "move" and _ != 0)
d, s = first(log, "doze"), first(log, "sleep")
off = any(x == ("perceive", False) for *_, a in log for x in a)
check("1 아무도 없음: 두리번→꾸벅→잠", d and 59 <= d <= 61 and s and 239 <= s <= 242 and off, f"꾸벅 {d}s 잠 {s}s 처리끔 {off}")

# 2. 새 사람 → 쳐다보고 끄덕 + LED 반짝
b, log = run(lambda t: [person(pan=20)] if t > 1 else [], 4)
tilts = [x[2] for *_, a in log for x in a if x[0] == "move"]
leds = [x[1] for *_, a in log for x in a if x[0] == "led"]
check("2 새 사람: 끄덕 인사", b.reason == "arrive" and min(tilts) <= 22.5 and 255 in leds, f"틸트들 {sorted(set(round(x) for x in tilts))} LED {leds}")

# 3. 사람이 컴퓨터만 봄 (yaw 60 고정) → 10~30 s 쳐다보다 딴짓 → 1 분 뒤 꾸벅 → 잠
b, log = run(lambda t: [person(yaw=60)], 300)
i, d, s = first(log, "idle"), first(log, "doze"), first(log, "sleep")
idle_pans = [x[1] for t, m, _, a in log if m == "idle" for x in a if x[0] == "move"]
check("3 무시하는 사람: 응시→딴짓→꾸벅→잠", i and 10 <= i <= 31 and d and 59 <= d <= 62 and s,
      f"딴짓 {i}s 꾸벅 {d}s 잠 {s}s")
check("3b 딴짓은 사람 곁(15~30°, 화면 안)", idle_pans and all(15 <= abs(p) <= 30 for p in idle_pans), [round(p) for p in idle_pans])

# 4. 딴짓 중 사람이 나를 쳐다봄 (yaw 60 → 5) → 바로 쳐다봄 (contact)
w = lambda t: [person(pan=10, yaw=60 if t < 40 else 5)]
b, log = run(w, 42)
r = [(t, m, rr) for t, m, rr, _ in log if t >= 40]
check("4 눈 맞추면 쳐다봄", any(m == "attend" and rr == "contact" for _, m, rr in r), r[-1])

# 5. 두리번 (1 s 안에 yaw 50→85, 나를 보진 않음) → 3 s 만 힐끗
w = lambda t: [person(yaw=50 if t < 40 else (50 + (t - 40) * 50 if t < 40.7 else 85))]
lb.GLANCE_ON = True
b, log = run(w, 48)
lb.GLANCE_ON = False
g = [(t, m, rr) for t, m, rr, _ in log if rr == "glance"]
gt = g[0][0] if g else 99
gi = first([x for x in log if x[0] > gt], "idle")
check("5 두리번 → 힐끗 3 s (켰을 때)", g and gi and 2.5 <= gi - gt <= 3.5, f"힐끗 {g[0][0] if g else None}s → 딴짓 {gi}s")

# 6. 다가옴 (얼굴 20→30 px, 1 s) → 쳐다봄
w = lambda t: [person(yaw=60, px=20 if t < 40 else min(30, 20 + (t - 40) * 10))]
b, log = run(w, 42)
check("6 다가오면 쳐다봄", any(rr == "approach" for t, m, rr, _ in log if t >= 40))

# 7. 잠 → LiDAR 움직임 → 깸 (얼굴 처리 켬, LED 반짝, 두리번)
b, log = run(lambda t: [], 245)
b, log = run(lambda t: [], 3, b, motion=lambda t: t > 246, t0=245)
acts = [x for *_, a in log for x in a]
check("7 잠 → LiDAR 로 깸", ("perceive", True) in acts and ("led", 255) in acts and b.mode == "scan", modes(log))

# 8. 꾸벅 중 사람이 나를 쳐다봄 → 번쩍 깨서 쳐다봄
w = lambda t: [person(yaw=60 if t < 100 else 5)]
b, log = run(w, 102)
r = [(t, m, rr) for t, m, rr, _ in log if t >= 99]
check("8 꾸벅 중 눈 맞춤 → 깸", r[0][1] == "doze" and r[-1][1] == "attend" and r[-1][2] == "contact", (r[0], r[-1]))

# 9. 쳐다보는 중 사람이 20° 이동 → 두 번 확인 후 한 번에 확 / 5° 이동은 가만히
w = lambda t: [person(pan=0 if t < 3 else 20, yaw=5)]
b, log = run(w, 5)
mv = [(t, round(x[1])) for t, m, _, a in log if t >= 3 for x in a if x[0] == "move"]
check("9a 크게 움직이면 도약", mv and mv[0][1] == 20, mv)
w = lambda t: [person(pan=0 if t < 3 else 5, yaw=5)]
b, log = run(w, 5)
mv = [(t, round(x[1], 1)) for t, m, _, a in log if t >= 3 for x in a if x[0] == "move" and x[2] > 20]
check("9b 조금(5°) 움직이면 도약 없이 다듬기 한 번", len(mv) == 1 and 4 <= mv[0][1] <= 5.5, mv)

# 10. LiDAR 움직임 감지
m = lb.Motion()
base = [3.0] * 360
quiet = [m.step(t, base) for t in range(5)]
near = base[:]
near[10:20] = [1.0] * 10
hit = [m.step(10 + t, near) for t in range(2)]
far = base[:]
far[50:60] = [2.5] * 10
m2 = lb.Motion()
m2.step(0, base)
far_hit = [m2.step(1 + t, far) for t in range(3)]
check("10 LiDAR: 가까운 움직임만", not any(quiet) and hit[-1] and not any(far_hit), f"{quiet} {hit} {far_hit}")

# 11. 실측 재현: yaw 가 1 s 마다 -8 ↔ -52 ↔ -90 으로 튐 (얼굴 22 px) → 쳐다보는 시간이 안 늘어남, 30 s 안에 딴짓
seq = [-8, -52, -90, -20, -90, -8, -54]
w = lambda t: [person(yaw=seq[int(t) % len(seq)], px=22)]
b, log = run(w, 40)
i = first(log, "idle")
trig = sorted({rr for t, m, rr, _ in log})
check("11 튀는 yaw 로 영원히 쳐다보지 않음", i and i <= 31, f"딴짓 {i}s, 이유들 {trig}")

# 12. 작은 얼굴(15 px) 은 yaw 가 바뀌어도 눈맞춤 아님 (못 믿음)
w = lambda t: [person(yaw=60 if t < 40 else 5, px=15)]
b, log = run(w, 42)
check("12 작은 얼굴 yaw 무시", not any(rr == "contact" for t, m, rr, _ in log if t >= 40), b.reason)

# 13. 딴짓하다 1 s 만 딴 데 봤다 다시 보면 contact 아님, 3 s 딴 데 봤다 보면 contact
w = lambda t: [person(yaw=5 if t < 35 else (60 if t < 36 else 5))]
b, log = run(w, 38)
short = any(rr == "contact" and t >= 35 for t, m, rr, _ in log)
w = lambda t: [person(yaw=5 if t < 35 else (60 if t < 38 else 5))]
b, log = run(w, 40)
long_ = any(rr == "contact" and t >= 38 for t, m, rr, _ in log)
check("13 잠깐 딴 데 → 아님 / 3 s 딴 데 → 눈맞춤", not short and long_, f"1s {short} 3s {long_}")

# 14. 눈맞춤 다듬기: 오차가 ±1° 로 흔들리기만 하면 안 움직임 (덜덜 없음)
import math
w = lambda t: [person(pan=1.0 * math.sin(t * 7), tilt=30 + 1.0 * math.cos(t * 5), yaw=5)]
b, log = run(w, 20)
mv = [t for t, m, _, a in log if t >= 2 for x in a if x[0] == "move" and m == "attend"]
check("14 작은 흔들림엔 가만히", not mv, mv)

# 15. 딴짓 중 사람을 놓쳐도 마지막 본 곳 곁에서만 (멀어지지 않음)
w = lambda t: [person(pan=-16, yaw=60)] if t < 20 else []
b, log = run(w, 26)
pans = [round(x[1]) for t, m, _, a in log if m == "idle" for x in a if x[0] == "move"]
check("15 놓쳐도 마지막 곳 곁", pans and all(abs(p + 16) <= 30 for p in pans), pans)

# 16. 먼 얼굴 크기 흔들림 (15↔21 px) 은 다가옴 아님 / 진짜 다가옴 (15→30) 은 다가옴
w = lambda t: [person(px=15 if int(t * 7) % 2 else 21, yaw=60)]
b, log = run(w, 40)
noisy = any(rr == "approach" for t, m, rr, _ in log)
w = lambda t: [person(px=15 if t < 30 else min(30, 15 + (t - 30) * 15), yaw=60)]
b, log = run(w, 33)
real = any(rr == "approach" for t, m, rr, _ in log)
check("16 크기 흔들림 ≠ 다가옴", not noisy and real, f"흔들림 {noisy} 진짜 {real}")

print(f"\n{sum(results)}/{len(results)} 통과")
sys.exit(0 if all(results) else 1)
