#!/usr/bin/env python3
"""executive_brain + lidar_motion 시뮬레이션 (ROS 없이). python3 tests/executive_sim.py"""
import os
import random
import sys

R = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "nodes")
sys.path.insert(0, os.path.join(R, "behavior"))
sys.path.insert(0, os.path.join(R, "perception"))
import executive_brain as eb  # noqa: E402
from lidar_motion import Motion  # noqa: E402

DT = 0.1
results = []


def check(name, ok, info=""):
    results.append(bool(ok))
    print(f"{'OK ' if ok else 'XX '} {name} {info}")


def run(secs, ex=None, t0=0.0, events=lambda t: [], people=lambda t: False, lidar=lambda t: False, req=lambda t: []):
    ex = ex or eb.Executive(t0)
    t = t0
    while t < t0 + secs - 1e-9:
        ex.tick(t, events(t), people(t), lidar(t), req(t))
        t = round(t + DT, 6)
    return ex


def when(ex, to):
    return next((t for t, a, b, w in ex.log if b == to), None)


def once(at):
    """at 초 한 번만 (0.1 s 칸)"""
    return lambda t: abs(t - at) < DT / 2


# 1. 아무 일 없음 → 60 s 꾸벅 → +180 s 잠
ex = run(260)
check("1 조용: 꾸벅 60 s → 잠 240 s", when(ex, "DOZE") and abs(when(ex, "DOZE") - 60) < 0.3 and abs(when(ex, "SLEEP") - 240) < 0.3,
      ex.log)

# 2. 사람이 있어도 상호작용이 없으면 똑같이 꾸벅 (있음 ≠ 이유)
ex = run(70, people=lambda t: True)
check("2 사람만 있음 → 그래도 꾸벅", when(ex, "DOZE") is not None, ex.log)

# 3. 눈맞춤 → COMPANION → 2분 조용 → IDLE
ev = once(10)
ex = run(200, events=lambda t: [("contact", 3)] if ev(t) else [])
c, i = when(ex, "COMPANION"), when(ex, "IDLE")
check("3 눈맞춤 → 같이 있기 → 2분 뒤 IDLE", c and abs(c - 10) < 0.2 and i and abs(i - 130) < 0.3, ex.log)

# 4. 같이 있는 중 이벤트가 또 오면 2분 연장
ex = run(260, events=lambda t: [("contact", 3)] if once(10)(t) or once(100)(t) else [])
check("4 이벤트마다 2분 연장", abs(when(ex, "IDLE") - 220) < 0.3, ex.log)

# 5. 꾸벅 중 다가옴 → 바로 COMPANION
ex = run(80, events=lambda t: [("approach", 1)] if once(75)(t) else [])
check("5 꾸벅 중 다가옴 → 같이 있기", [x[1:3] for x in ex.log] == [("IDLE", "DOZE"), ("DOZE", "COMPANION")], ex.log)

# 6. 잠 → LiDAR → IDLE → 얼굴 없으면 10 s 뒤 다시 꾸벅
ex = run(250)
ex = run(30, ex, t0=250, lidar=once(251))
w, d = when(ex, "IDLE"), [t for t, a, b, why in ex.log if b == "DOZE" and t > 250]
check("6 LiDAR 로 깸 → 아무도 없으면 10 s 뒤 꾸벅", w and abs(w - 251) < 0.2 and d and abs(d[0] - 261) < 0.3, ex.log[-2:])

# 6b. LiDAR 로 깼는데 얼굴을 봄 → 보통 규칙 (60 s)
ex = run(250)
ex = run(80, ex, t0=250, lidar=once(251), people=lambda t: 252 < t < 253)
d = [t for t, a, b, why in ex.log if b == "DOZE" and t > 250]
check("6b LiDAR 로 깨고 얼굴 봄 → 60 s 규칙", d and abs(d[0] - 311) < 0.3, ex.log[-2:])

# 7. 잠 중엔 얼굴·이벤트로 안 깸 (처리 꺼져 있음), LiDAR 만
ex = run(245)
ex = run(10, ex, t0=245, events=lambda t: [("contact", 1)], people=lambda t: True)
check("7 잠 중 이벤트 무시", ex.mode == "SLEEP", ex.mode)

# 8. 요청 sleep / wake
ex = run(5, req=lambda t: ["sleep"] if once(2)(t) else [])
ex = run(5, ex, t0=5, req=lambda t: ["wake"] if once(7)(t) else [])
check("8 요청 sleep → wake", [x[2] for x in ex.log] == ["SLEEP", "IDLE"], ex.log)

# 9. 예약 작업: IDLE 이면 바로 / COMPANION 이면 끝날 때 (2분 안에 끝나면)
ex = run(5, req=lambda t: ["task:patrol"] if once(2)(t) else [])
check("9a IDLE 중 작업 → 바로", ex.mode == "TASK:patrol", ex.log)
ex = run(200, events=lambda t: [("contact", 1)] if once(1)(t) else [], req=lambda t: ["task:patrol"] if once(30)(t) else [])
check("9b 같이 있는 중 작업 → 같이 있기 끝나면 (121 s)", abs(when(ex, "TASK:patrol") - 121) < 0.3, ex.log)

# 9c. 계속 같이 있으면 최대 10분 미루고 감
ex = run(700, events=lambda t: [("contact", 1)] if int(t * 10) % 600 == 0 else [], req=lambda t: ["task:patrol"] if once(30)(t) else [])
check("9c 계속 같이 있어도 10분이면 감 (630 s)", abs(when(ex, "TASK:patrol") - 630) < 0.3, ex.log)

# 9d. 작업 중엔 이벤트가 와도 작업 계속, done 이면 IDLE
ex = run(20, req=lambda t: ["task:patrol"] if once(1)(t) else (["done:patrol"] if once(15)(t) else []),
         events=lambda t: [("contact", 1)] if once(5)(t) else [])
check("9d 작업 중 이벤트 무시 → done 이면 IDLE", [x[2] for x in ex.log] == ["TASK:patrol", "IDLE"], ex.log)

# 10. 설정표: 모드마다 칸
ex = eb.Executive()
p = {m: (eb.PROFILE[m]["face_hz"], eb.PROFILE[m]["head"]) for m in ("SLEEP", "IDLE", "DOZE", "COMPANION")}
check("10 설정표", p == {"SLEEP": (0.0, "sleep"), "IDLE": (2.0, "free"), "DOZE": (2.0, "doze"), "COMPANION": (5.0, "free")}
      and ex.profile()["mode"] == "IDLE", p)

# 11. 시간을 바꾸면 그대로 따름 (나중에 랜덤으로 갈아끼울 자리)
eb.DUR["doze_after"] = 20.0
ex = run(25)
eb.DUR["doze_after"] = 60.0
check("11 dur() 한 곳만 바꾸면 됨", abs(when(ex, "DOZE") - 20) < 0.3, ex.log)

# 12. LiDAR "새로 들어옴" (look_brain 에서 이사 — 옆사람 꼼지락·모서리·먼 변화 무시, 새로 들어온 사람만)
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
check("12 LiDAR: 새로 들어온 사람만", not any(fired) and not any(far_hit) and near_hit[-1], f"빔 {m.last}")

print(f"\n{sum(results)}/{len(results)} 통과")
sys.exit(0 if all(results) else 1)
