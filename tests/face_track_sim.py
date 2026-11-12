#!/usr/bin/env python3
"""face_track 시뮬레이션 (ROS 없이). python3 tests/face_track_sim.py
   가짜 얼굴 방향을 시간에 따라 만들어 넣고, 트랙 번호가 기대대로 유지·생성되는지 확인."""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "scripts"))
from face_track import Tracker, face_angles, yaw_pitch  # noqa: E402

DT = 0.1   # 10 fps


def det(pan, tilt, px=30.0):
    return {"pan": pan, "tilt": tilt, "px": px, "det": 0.9, "yaw": 0.0, "pitch": 0.0}


def ids(tr):
    return [t.id for t in tr.confirmed()]


results = []


def check(name, ok, info=""):
    results.append(ok)
    print(f"{'OK ' if ok else 'XX '} {name} {info}")


# 1. 가만히 있는 한 명 → 번호 하나 유지
tr = Tracker()
for k in range(50):
    tr.step([det(10, 20)], k * DT)
check("1 가만히 한 명", ids(tr) == [1], ids(tr))

# 2. 천천히 40° 옆으로 걸어감 (초당 10°) → 같은 번호
tr = Tracker()
for k in range(40):
    tr.step([det(-20 + k * 1.0, 20)], k * DT)
check("2 천천히 이동", ids(tr) == [1], ids(tr))

# 3. 고개가 30° 확 돎: 화면 속 얼굴은 튀지만 방향(고개+화면)은 같음 → 같은 번호
tr = Tracker()
for k in range(30):
    head = 0.0 if k < 15 else 30.0
    ex, _ = face_angles(160 + (35 - head) / 60 * 160, 120, 320, 240)   # 사람은 방향 35° 에 서 있음
    tr.step([det(head + ex, 20)], k * DT)
check("3 고개 도약에도 유지", ids(tr) == [1], ids(tr))

# 4. 두 사람이 가까이 (방향 10° 차이), 하나는 가깝고(얼굴 크게) 하나는 멀리 → 번호 둘, 바뀌지 않음
tr = Tracker()
for k in range(40):
    a = 0 + k * 0.25          # 가까운 사람 오른쪽으로
    b = 10 - k * 0.25         # 먼 사람 왼쪽으로 → 서로 지나침
    tr.step([det(a, 20, 50), det(b, 20, 15)], k * DT)
big = max(tr.confirmed(), key=lambda t: t.px)
check("4 두 사람 엇갈림", sorted(ids(tr)) == [1, 2] and big.id == 1, f"{ids(tr)} 큰얼굴={big.id}")

# 5a. 1 s 사라졌다 다시 → 같은 번호
tr = Tracker()
t = 0.0
for k in range(10):
    tr.step([det(0, 20)], t); t += DT
for k in range(10):
    tr.step([], t); t += DT
for k in range(5):
    tr.step([det(2, 20)], t); t += DT
check("5a 1 s 가림 → 같은 번호", ids(tr) == [1], ids(tr))

# 5b. 2 s 사라졌다 다시 → 새 번호
tr = Tracker()
t = 0.0
for k in range(10):
    tr.step([det(0, 20)], t); t += DT
for k in range(20):
    tr.step([], t); t += DT
for k in range(5):
    tr.step([det(2, 20)], t); t += DT
check("5b 2 s 가림 → 새 번호", ids(tr) == [2], ids(tr))

# 6. 한 장짜리 헛것 → 알리지 않고 사라짐
tr = Tracker()
seen = set()
for k in range(30):
    d = [det(0, 20)] + ([det(-50, 0)] if k == 10 else [])
    tr.step(d, k * DT)
    seen.update(ids(tr))
check("6 헛것 한 장 무시", seen == {1}, seen)

# 7. 얼굴 방향 부호: 코가 화면 오른쪽 → yaw +, 코가 눈 쪽으로 → pitch +
front = [(100, 100), (140, 100), (120, 122), (105, 140), (135, 140)]
left = [(100, 100), (140, 100), (135, 122), (105, 140), (135, 140)]
up = [(100, 100), (140, 100), (120, 110), (105, 140), (135, 140)]
y0, p0 = yaw_pitch(front)
y1, _ = yaw_pitch(left)
_, p2 = yaw_pitch(up)
check("7 얼굴 방향 부호", abs(y0) < 5 and abs(p0) < 10 and y1 > 20 and p2 > 20,
      f"정면 {y0:.0f}/{p0:.0f} 왼쪽 {y1:.0f} 위 {p2:.0f}")

print(f"\n{sum(results)}/{len(results)} 통과")
sys.exit(0 if all(results) else 1)
