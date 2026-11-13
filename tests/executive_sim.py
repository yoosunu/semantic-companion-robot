#!/usr/bin/env python3
"""executive_brain + lidar_motion + imu_touch 시뮬레이션 (ROS 없이). python3 tests/executive_sim.py"""
import math
import os
import random
import sys

R = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "nodes")
sys.path.insert(0, os.path.join(R, "behavior"))
sys.path.insert(0, os.path.join(R, "perception"))
import executive_brain as eb  # noqa: E402
from imu_touch import Touch  # noqa: E402
from lidar_motion import Motion  # noqa: E402

DT = 0.1
results = []


def check(name, ok, info=""):
    results.append(bool(ok))
    print(f"{'OK ' if ok else 'XX '} {name} {info}")


def run(secs, ex=None, t0=0.0, events=lambda t: [], people=lambda t: False, stim=lambda t: [], req=lambda t: [], seed=1):
    ex = ex or eb.Executive(t0, random.Random(seed))
    t = t0
    while t < t0 + secs - 1e-9:
        ex.tick(t, events(t), people(t), stim(t), req(t))
        t = round(t + DT, 6)
    return ex


def when(ex, to, after=-1):
    return next((t for t, a, b, w in ex.log if b == to and a != b and t > after), None)


def once(at):
    return lambda t: abs(t - at) < DT / 2


def asleep_at(at):
    return lambda t: [("asleep", -1)] if once(at)(t) else []


# 1. 조용: 30~60 s 뒤 꾸벅 (랜덤), look 이 asleep 보내면 잠
ds = []
for seed in range(20):
    ex = run(70, seed=seed)
    ds.append(when(ex, "DOZE"))
check("1 조용: 꾸벅 시작 30~60 s 사이, 매번 다름", all(d and 30 <= d <= 60.2 for d in ds) and len(set(ds)) > 10,
      f"{min(ds)}~{max(ds)}")
ex = run(70, seed=3)
d = when(ex, "DOZE")
ex = run(60, ex, t0=70, events=asleep_at(100))
check("1b 꾸벅 다 함(asleep) → 잠", when(ex, "SLEEP") == 100.0, ex.log)

# 1c. asleep 이 안 오면 90 s 뒤 잠 (안전장치)
ex = run(200, seed=3)
check("1c asleep 안 와도 90 s 뒤 잠", abs(when(ex, "SLEEP") - (d + 90)) < 0.3, ex.log)

# 2. 사람이 있어도 상호작용이 없으면 꾸벅 (있음 ≠ 이유)
ex = run(70, people=lambda t: True)
check("2 사람만 있음 → 그래도 꾸벅", when(ex, "DOZE") is not None, ex.log)

# 3. 눈맞춤 → COMPANION → look 이 쳐다보기 끝(bored) → 5 s 안에 다시 없으면 IDLE
ev = lambda t: [("contact", 3)] if once(10)(t) else ([("bored", 3)] if once(30)(t) else [])
ex = run(60, events=ev)
check("3 눈맞춤 → 같이 있기 → bored + 5 s → IDLE", when(ex, "COMPANION") == 10.0 and abs(when(ex, "IDLE") - 35) < 0.2, ex.log)

# 3b. bored 뒤 5 s 안에 다시 눈맞춤 → 계속 같이 있기
ev = lambda t: [("contact", 3)] if once(10)(t) or once(32)(t) else ([("bored", 3)] if once(30)(t) else [])
ex = run(50, events=ev)
check("3b bored 뒤 다시 눈맞춤 → 계속", when(ex, "IDLE") is None and ex.mode == "COMPANION", ex.log)

# 3c. bored 가 안 와도 40 s 뒤 IDLE (안전장치)
ex = run(60, events=lambda t: [("contact", 3)] if once(10)(t) else [])
check("3c bored 없어도 40 s 뒤 IDLE", abs(when(ex, "IDLE") - 50) < 0.2, ex.log)

# 4. 꾸벅 중 다가옴 → 바로 COMPANION + 깨움 표시
ex = run(70, seed=3)
ex = run(10, ex, t0=70, events=lambda t: [("approach", 1)] if once(75)(t) else [])
check("4 꾸벅 중 다가옴 → 같이 있기 (깨움)", ex.mode == "COMPANION" and ex.wake and ex.wake["strength"] == "strong", ex.log[-1])

# 5. 잠의 깊이 vs 자극 세기
def sleep_now(seed=1):
    return run(1, req=lambda t: ["sleep"] if once(0)(t) else [], seed=seed)


ex = sleep_now()
ex = run(10, ex, t0=1, stim=lambda t: [("lidar", 1, 30.0)] if once(5)(t) else [])
check("5a 얕은 잠 + 약한 자극(LiDAR) → 깸 (천천히)", ex.mode == "IDLE" and ex.wake["strength"] == "weak" and ex.wake["dir"] == 30.0, ex.wake)
ex = sleep_now()
ex = run(200, ex, t0=1, stim=lambda t: [("lidar", 1, 30.0)] if once(150)(t) else [])
check("5b 깊은 잠 + 약한 자극 → 무시", ex.mode == "SLEEP" and ex.stir == 0, ex.mode)
ex = sleep_now()
ex = run(200, ex, t0=1, stim=lambda t: [("lidar", 2, 30.0)] if once(150)(t) else [])
check("5c 깊은 잠 + 중간 자극 → 뒤척임만", ex.mode == "SLEEP" and ex.stir == 1, (ex.mode, ex.stir))
ex = sleep_now()
ex = run(151, ex, t0=1, stim=lambda t: [("touch", 3, -60.0)] if once(150)(t) else [])
check("5d 깊은 잠 + 톡톡(강) → 번쩍, 맞은 쪽으로", ex.mode == "IDLE" and ex.wake["strength"] == "strong" and ex.wake["dir"] == -60.0, ex.wake)

# 6. 자극으로 깼는데 얼굴 없음 → 10 s 뒤 다시 꾸벅 / 얼굴 봄 → 30~60 s 규칙
ex = sleep_now()
ex = run(30, ex, t0=1, stim=lambda t: [("touch", 3, None)] if once(5)(t) else [])
check("6 깼는데 아무도 없음 → 10 s 뒤 꾸벅", abs(when(ex, "DOZE", 5) - 15) < 0.2, ex.log[-2:])
ex = sleep_now()
ex = run(80, ex, t0=1, stim=lambda t: [("touch", 3, None)] if once(5)(t) else [], people=lambda t: 6 < t < 7)
d = when(ex, "DOZE", 5)
check("6b 깨고 얼굴 봄 → 30~60 s 규칙", d and 36 <= d <= 67.2, ex.log[-2:])

# 7. 꾸벅 중 어떤 자극이든 깸
ex = run(70, seed=3)
ex = run(5, ex, t0=70, stim=lambda t: [("lidar", 1, 10.0)] if once(72)(t) else [])
check("7 꾸벅 중 약한 자극도 깸", ex.mode == "IDLE" and ex.wake["dir"] == 10.0, ex.log[-1])

# 8. 요청 sleep / wake
ex = run(5, req=lambda t: ["sleep"] if once(2)(t) else [])
ex = run(5, ex, t0=5, req=lambda t: ["wake"] if once(7)(t) else [])
check("8 요청 sleep → wake", [x[2] for x in ex.log] == ["SLEEP", "IDLE"], ex.log)

# 9. 예약 작업
ex = run(5, req=lambda t: ["task:patrol"] if once(2)(t) else [])
check("9a IDLE 중 작업 → 바로", ex.mode == "TASK:patrol", ex.log)
ev = lambda t: [("contact", 1)] if once(1)(t) else ([("bored", 1)] if once(20)(t) else [])
ex = run(40, events=ev, req=lambda t: ["task:patrol"] if once(10)(t) else [])
check("9b 같이 있는 중 작업 → 같이 있기 끝나면 (25 s)", abs(when(ex, "TASK:patrol") - 25) < 0.2, ex.log)
ex = run(700, events=lambda t: [("contact", 1)] if int(round(t * 10)) % 300 == 0 else [], req=lambda t: ["task:patrol"] if once(30)(t) else [])
check("9c 계속 같이 있어도 10분이면 감 (630 s)", abs(when(ex, "TASK:patrol") - 630) < 0.2, ex.log[-1:])
ex = run(20, req=lambda t: ["task:patrol"] if once(1)(t) else (["done:patrol"] if once(15)(t) else []),
         events=lambda t: [("contact", 1)] if once(5)(t) else [])
check("9d 작업 중 이벤트 무시 → done 이면 IDLE", [x[2] for x in ex.log] == ["TASK:patrol", "IDLE"], ex.log)

# 10. 설정표
ex = eb.Executive()
p = {m: (eb.PROFILE[m]["face_hz"], eb.PROFILE[m]["head"]) for m in ("SLEEP", "IDLE", "DOZE", "COMPANION")}
check("10 설정표 + 꾸벅 수", p == {"SLEEP": (0.0, "sleep"), "IDLE": (2.0, "free"), "DOZE": (2.0, "doze"), "COMPANION": (5.0, "free")}
      and ex.profile()["nods"] == 3, p)

# 11. DUR 한 곳만 바꾸면 그대로 따름
eb.DUR["doze_after"] = (20.0, 20.0)
ex = run(25)
eb.DUR["doze_after"] = (30.0, 60.0)
check("11 DUR 한 곳만 바꾸면 됨", abs(when(ex, "DOZE") - 20) < 0.2, ex.log)

# 12. LiDAR "새로 들어옴" + 들어온 방향
rng = random.Random(3)


def scan(t, enter=False, far=False):
    r = [3.0 + rng.uniform(-0.05, 0.05) for _ in range(360)]
    r[100] = 1.0 if int(t * 10) % 2 else 3.0
    r[200:205] = [0.2] * 5
    r[50:62] = [rng.uniform(0.6, 1.1) for _ in range(12)]
    if enter:
        r[120:132] = [1.0] * 12
    if far:
        r[150:162] = [2.5] * 12
    return r


m = Motion()
t, fired = 0.0, []
while t < 20:
    fired.append(m.step(t, scan(t)))
    t += 0.1
far_hit = [m.step(t + k * 0.1, scan(t, far=True)) for k in range(5)]
near_hit = [m.step(t + 1 + k * 0.1, scan(t, enter=True)) for k in range(4)]
check("12 LiDAR: 새로 들어온 사람만 + 방향", not any(fired) and not any(far_hit) and near_hit[-1] and abs(m.last_idx - 125.5) < 1,
      f"빔 {m.last}, 가운데 {m.last_idx}")

# 13. 톡톡: 기준 잡고 → 톡 0.04 g → 감지 + 방향 / 고개 움직인 직후(quiet=False) 는 무시 / 1 s 안 두 번은 한 번
G = 9.81
tc = Touch()
rng = random.Random(4)
hits = []
for k in range(400):
    t = k * 0.05
    ax = rng.gauss(0, 0.004) * G
    ay = rng.gauss(0, 0.004) * G
    quiet = not (5 <= t < 6)
    if 5.5 <= t < 5.6:
        ax += 0.05 * G                                  # 고개 돌며 흔들림 → 무시돼야
    if 10.0 <= t < 10.1 or 10.4 <= t < 10.5:
        ay -= 0.04 * G                                  # 왼쪽에서 톡 → 오른쪽(−y)으로 밀림, 두 번 (1 s 안)
    r = tc.step(t, ax, ay, 0.0, 0.0, quiet)
    if r:
        hits.append((round(t, 2), r[0] if r[0] is None else round(r[0])))
check("13 톡톡: 감지·방향(왼쪽 +90°)·자기 흔들림 무시·한 번만", len(hits) == 1 and hits[0][0] == 10.0 and abs(hits[0][1] - 90) < 15, hits)

print(f"\n{sum(results)}/{len(results)} 통과")
sys.exit(0 if all(results) else 1)
