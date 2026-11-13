#!/usr/bin/env python3
"""look_brain 시뮬레이션 (ROS 없이). python3 tests/look_brain_sim.py
   가짜 /people 을 시간에 따라 넣고, 모드·동작이 규칙대로 나오는지 확인."""
import os
import random
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "nodes", "behavior"))
import look_brain as lb  # noqa: E402

DT = 0.1
results = []


def check(name, ok, info=""):
    results.append(bool(ok))
    print(f"{'OK ' if ok else 'XX '} {name} {info}")


def person(pan=0.0, tilt=30.0, px=25.0, yaw=60.0, pid=1, unseen=0.0):
    return {"id": pid, "pan": pan, "tilt": tilt, "px": px, "yaw": yaw, "pitch": 0.0, "unseen": unseen}


def run(world, secs, b=None, head=lambda t: "free", t0=0.0, moving=lambda t: False):
    """world(t) → people 목록, head(t) → executive 가 시킨 고개 상태. 기록: [(t, 모드, 이유, 동작들)]"""
    b = b or lb.Brain(random.Random(1))
    log = []
    t = t0
    while t < t0 + secs:
        acts = b.set_head(head(t), t) + b.tick(t, world(t), moving(t))
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


# 1. 고개 상태는 executive 가 정함: doze → 꾸벅 자세 / sleep → 끝까지 숙임 / free → 번쩍 (11-14 사령관 v1)
H = lambda t: "doze" if 10 <= t < 30 else ("sleep" if 30 <= t < 40 else "free")
b, log = run(lambda t: [], 45, head=H)
seq = [m for _, m in modes(log)]
tilts = {round(t): round(x[2]) for t, m, _, a in log for x in a if x[0] == "move"}
check("1 head 따라: 두리번→꾸벅→잠→번쩍", seq[:4] == ["scan", "doze", "sleep", "scan"] and tilts.get(30) == -30
      and any(x == ("led", 255) for t, m, _, a in log if t >= 40 for x in a), f"{modes(log)}")

# 1b. executive 가 없으면(head 계속 free) 스스로 졸지 않음 — 타이머는 이사 갔음
b, log = run(lambda t: [person(yaw=60)], 300)
check("1b 혼자서는 안 졸음", first(log, "doze") is None and first(log, "sleep") is None, modes(log)[-3:])

# 2. 새 사람 → 쳐다보고 끄덕 + LED 반짝
b, log = run(lambda t: [person(pan=20)] if t > 1 else [], 4)
tilts = [x[2] for *_, a in log for x in a if x[0] == "move"]
leds = [x[1] for *_, a in log for x in a if x[0] == "led"]
check("2 새 사람: 끄덕 인사", b.reason == "arrive" and min(tilts) <= 22.5 and 255 in leds, f"틸트들 {sorted(set(round(x) for x in tilts))} LED {leds}")

# 3. 사람이 컴퓨터만 봄 (yaw 60 고정) → 10~30 s 쳐다보다 딴짓 (꾸벅·잠은 executive 몫)
b, log = run(lambda t: [person(yaw=60)], 60)
i = first(log, "idle")
idle_pans = [x[1] for t, m, _, a in log if m == "idle" for x in a if x[0] == "move"]
check("3 무시하는 사람: 응시(5~30 s)→딴짓", i and 5 <= i <= 31, f"딴짓 {i}s")
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

# 7. 트리거는 executive 에 알림 (events) — 인사·눈맞춤·다가옴
w = lambda t: [person(pan=10, yaw=60 if t < 40 else 5)] if t > 1 else []
b, log = run(w, 42)
names = [e[0] for e in b.events]
check("7 트리거 → events 로 알림", "arrive" in names and "contact" in names, b.events)

# 8. 꾸벅 중 눈맞춤 → 스스로 안 깨고 알리기만 (깨우는 건 executive → head=free)
w = lambda t: [person(yaw=60 if t < 100 else 5)]
b, log = run(w, 102, head=lambda t: "doze" if t >= 60 else "free")
check("8 꾸벅 중 눈맞춤 → 알림만, 계속 꾸벅", b.mode == "doze" and ("contact", 1) in b.events, (b.mode, b.events[-2:]))
b, log = run(w, 3, b, head=lambda t: "free", t0=102)
check("8b executive 가 free 로 → 번쩍 깨서 쳐다봄", b.mode == "attend", modes(log))

# 9. 쳐다보는 중 사람이 20° 이동 → 두 번 확인 후 한 번에 확 / 5° 이동은 가만히
w = lambda t: [person(pan=0 if t < 3 else 20, yaw=5)]
b, log = run(w, 5)
mv = [(t, round(x[1])) for t, m, _, a in log if t >= 3 for x in a if x[0] == "move"]
check("9a 크게 움직이면 도약", mv and mv[0][1] == 20, mv)
w = lambda t: [person(pan=0 if t < 3 else 5, yaw=5)]
b, log = run(w, 5)
mv = [(t, round(x[1], 1)) for t, m, _, a in log if t >= 3 for x in a if x[0] == "move" and x[2] > 20]
check("9b 조금(5°) 움직이면 도약 없이 다듬기 한 번", len(mv) == 1 and 4 <= mv[0][1] <= 5.5, mv)

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

# 17. 로봇이 움직이는 중 얼굴이 커짐 → 다가옴 아님 (우리가 다가간 것)
w = lambda t: [person(px=15 if t < 30 else min(30, 15 + (t - 30) * 15), yaw=60)]
b, log = run(w, 33, moving=lambda t: t >= 29)
check("17 로봇이 움직이면 다가옴 아님", not any(rr == "approach" for t, m, rr, _ in log))

# 18. 꾸벅 모양 (11-14): 스르르륵(느림) → 툭(빠름, 더 아래) → 깸(빠름, 졸린 자세보다 위) / 꾸벅 사이 아주 느리게 가라앉음
shapes = []
for seed in range(6):
    b = lb.Brain(random.Random(seed))
    log = []
    t = 0.0
    while t < 30:
        acts = b.set_head("doze" if t >= 1 else "free", t, nods=3) + b.tick(t, [])
        log += [(round(t, 1), x) for x in acts if x[0] == "move"]
        t += DT
    mv = [x for t, x in log if t > 2.5]
    slow = next((i for i, x in enumerate(mv) if 0 < x[3] < 60), None)       # 스르르륵 (느림)
    ok = slow is not None and slow + 2 < len(mv)
    if ok:
        s1, drop, up = mv[slow], mv[slow + 1], mv[slow + 2]
        ok = drop[3] == 0 and drop[2] < s1[2] and up[3] == 0 and up[2] > lb.TILT_DOZE
    drift = any(x[3] == 10 for x in mv)                                    # 사이 가라앉음
    shapes.append(ok and drift)
check("18 꾸벅: 스르르륵→툭→깸(위로)→느린 가라앉음, A·B 섞여도", all(shapes), shapes)

# 18b. 꾸벅 3번째는 안 깨고 끝까지 숙임 → asleep 알림
b = lb.Brain(random.Random(2))
t, ev_t = 0.0, None
while t < 120 and ev_t is None:
    b.set_head("doze" if t >= 1 else "free", t, nods=3)
    b.tick(t, [])
    if any(e[0] == "asleep" for e in b.events):
        ev_t = round(t, 1)
    t += DT
check("18b 꾸벅 3번 → 끝까지 숙이고 asleep", ev_t and b.tilt == lb.TILT_SLEEP and b.nods_left == 0, f"{ev_t}s 틸트 {b.tilt}")

# 19. 깨는 모양: strong = 빠르게 자극 쪽(팬 −60) / weak = 천천히
b = lb.Brain(random.Random(1))
b.set_head("sleep", 0)
acts = b.set_head("free", 5, wake={"id": 1, "strength": "strong", "dir": -60.0})
mv = [x for x in acts if x[0] == "move"]
b2 = lb.Brain(random.Random(1))
b2.set_head("sleep", 0)
mv2 = [x for x in b2.set_head("free", 5, wake={"id": 1, "strength": "weak", "dir": None}) if x[0] == "move"]
check("19 깨기: 번쩍 자극 쪽 / 천천히", mv and round(mv[0][1]) == -60 and mv[0][3] == 0 and mv2 and mv2[0][3] > 0, (mv, mv2))

# 20. 뒤척임: 잠 중 stir 가 바뀌면 살짝 움직였다 다시 잠 자세
b = lb.Brain(random.Random(1))
b.set_head("sleep", 0)
log = []
t = 0.0
while t < 8:
    log += [x for x in b.set_head("sleep", t, stir=1 if t >= 3 else 0) + b.tick(t, []) if x[0] == "move"]
    t += DT
check("20 뒤척임: 움직였다 잠 자세로, 계속 잠", len(log) >= 2 and log[-1][2] == lb.TILT_SLEEP and b.mode == "sleep", log)

# 21. 클립 ① 까꿍: 쳐다보던 얼굴이 1.5 s 사라졌다 같은 자리 → 놀람 + 끄덕 + 알림
w = lambda t: [] if 6.0 <= t < 7.5 else [person(pan=5, tilt=30, px=30, yaw=5)]
b, log = run(w, 9)
check("21 까꿍", ("peekaboo", 1) in b.events or any(e[0] == "peekaboo" for e in b.events), b.events)

# 21b. 얼굴 2 Hz (검출 사이 unseen 0~0.5 s) 는 까꿍 아님 (11-14 실물: 깨자마자 가짜 까꿍 두 번)
w = lambda t: [person(pan=5, tilt=30, px=30, yaw=5, unseen=round((t % 0.5), 2))]
b, log = run(w, 12)
check("21b 2 Hz 검출 사이는 까꿍 아님", not any(e[0] == "peekaboo" for e in b.events), b.events)

# 22. 클립 ② 시선 따라가기: 눈맞춤(yaw 5) → 고개 돌려(yaw 50) 1 s 유지 → 사람+30° 쪽 힐끗
w = lambda t: [person(pan=0, px=30, yaw=5 if t < 6 else 50)]
b, log = run(w, 9)
pans = [round(x[1]) for t, m, _, a in log if t >= 6 for x in a if x[0] == "move"]
check("22 시선 따라가기", any(e[0] == "follow_gaze" for e in b.events) and 30 in pans, (pans, b.events))

# 22b. 두리번(금방 돌아옴)은 따라가지 않음
w = lambda t: [person(pan=0, px=30, yaw=5 if (t < 6 or t > 6.6) else 50)]
b, log = run(w, 9)
check("22b 잠깐 돌린 건 안 따라감", not any(e[0] == "follow_gaze" for e in b.events), b.events)

# 23. 클립 ④ 움찔: 아주 빠른 다가옴 (25→50 px) → 뒤로 젖힘(틸트 +5) 뒤 올려다봄
w = lambda t: [person(px=25 if t < 20 else min(50, 25 + (t - 20) * 50), yaw=60)]
b, log = run(w, 22)
mv = [(t, round(x[2])) for t, m, _, a in log if t >= 20 for x in a if x[0] == "move"]
check("23 움찔: 젖힘(+5) → 올려다봄", any(e[0] == "flinch" for e in b.events) and any(t2 > t1 and a2 - a1 == 5 for (t1, a1), (t2, a2) in zip(mv, mv[1:])), mv[:4])

print(f"\n{sum(results)}/{len(results)} 통과")
sys.exit(0 if all(results) else 1)
