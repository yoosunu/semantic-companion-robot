#!/usr/bin/env python3
"""person_bind_sim — 얼굴과 몸 묶기 (nodes/perception/person_bind.py) 시험 (11-26). ROS 없음.
   python3 tests/person_bind_sim.py
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "nodes", "perception"))
import face_owner as fo  # noqa: E402
import person_bind as pb  # noqa: E402

results = []


def check(name, ok, info=""):
    results.append(bool(ok))
    print(("통과 " if ok else "실패 ") + name + (f"  ({info})" if info != "" else ""))


def body(pan, tilt=10.0, w=20.0, h=60.0):
    return {"pan": pan, "tilt": tilt, "w": w, "h": h}           # 몸 가운데 틸트 10°, 위 끝 40°


GOOD = (15.0, 0.85)
P = pb.PeopleBinder(fo.OwnerVote)

# 1~3. 나: 얼굴 보이고 이름 → 뒤돌기 (얼굴 사라짐) → 몸이 이름 유지
P.step_bodies([body(0.0)], 0.0)
check("1 몸 하나 생김", len(P.bodies) == 1)
P.bind([(7, 1.0, 32.0)], 0.0)
check("2 얼굴 (가운데 위쪽) → 그 몸에 묶임", P.face_body.get(7) == 1, P.face_body)
for c in (0.5, 0.5):
    P.vote_of(7).update(c, *GOOD)
check("3 얼굴 인식 2/3 → 몸 이름 jasper, 얼굴 이름도 jasper", P.bodies[1].name == "jasper" and P.name_of(7) == "jasper")
P.drop_faces(set())                                              # 뒤돎: 얼굴 추적 끝
for k, t in enumerate((0.5, 1.0, 1.5, 2.0, 2.5)):
    P.step_bodies([body(2.0 + k)], t)                            # 몸은 조금씩 움직이며 계속 보임
check("4 뒤돌아 얼굴 없어도 (2.5 s) 몸 이름 jasper 유지", P.bodies[1].name == "jasper" and P.bodies[1].face is None,
      (P.bodies[1].name, P.bodies[1].face))
P.bind([(9, 7.0, 33.0)], 2.5)
check("5 다시 돌아본 얼굴 (새 추적 #9) → 같은 몸, 바로 jasper (다시 모으지 않음)", P.name_of(9) == "jasper", P.name_of(9))

# 6. 몸 밖 얼굴 (다리 쪽·옆) 은 안 묶임
P.bind([(11, 7.0, -10.0)], 2.5)
check("6 몸 가운데보다 아래 얼굴 → 안 묶임 (얼굴은 윗부분)", 11 not in P.face_body)

# 7. 두 사람 나란히: 얼굴이 각자 몸에
Q = pb.PeopleBinder(fo.OwnerVote)
Q.step_bodies([body(-20.0), body(20.0)], 0.0)
Q.bind([(1, -19.0, 30.0), (2, 21.0, 31.0)], 0.0)
check("7 두 사람 나란히 → 얼굴이 각자 몸에", Q.face_body == {1: 1, 2: 2}, Q.face_body)
for c in (0.5, 0.6):
    Q.vote_of(1).update(c, *GOOD)
for c in (0.1, 0.12, 0.15, 0.1, 0.11):
    Q.vote_of(2).update(c, *GOOD)
check("8 왼쪽 jasper, 오른쪽 stranger — 서로 안 섞임", Q.bodies[1].name == "jasper" and Q.bodies[2].name == "stranger",
      (Q.bodies[1].name, Q.bodies[2].name))

# 9. 몸 없이 먼저 모은 얼굴 투표를 몸이 넘겨받음
R = pb.PeopleBinder(fo.OwnerVote)
for c in (0.5, 0.5):
    R.vote_of(4).update(c, *GOOD)                                # 몸 아직 없음 (YOLO 장 아님)
check("9a 몸 없는 얼굴도 자기 투표로 jasper", R.name_of(4) == "jasper")
R.step_bodies([body(0.0)], 1.0)
R.bind([(4, 0.0, 30.0)], 1.0)
check("9b 나중에 몸에 묶이면 그 투표를 몸이 넘겨받음 → 몸 jasper", R.bodies[1].name == "jasper")

# 10. 몸이 3 s 넘게 안 보이면 지움 (이름도 같이 사라짐 → 다음엔 다시 알아봐야 함)
R.step_bodies([], 4.5)
check("10 몸 3.5 s 안 보임 → 지움, 묶음도 정리", not R.bodies and 4 not in R.face_body)

# 11. 겹친 두 상자 (앞사람 큰 상자 안에 뒷사람 작은 상자) → 작은 쪽
S = pb.PeopleBinder(fo.OwnerVote)
S.step_bodies([body(0.0, w=40.0, h=90.0), body(3.0, tilt=15.0, w=12.0, h=40.0)], 0.0)
S.bind([(5, 3.0, 30.0)], 0.0)
check("11 겹친 상자 → 얼굴이 들어가는 가장 작은 상자", S.face_body.get(5) == 2, S.face_body)

# 12. 11-26 실물에서 노드가 죽음: YOLO 좌표가 numpy float32 → body_list 를 json 으로 못 씀
import json  # noqa: E402
import numpy as np  # noqa: E402
T = pb.PeopleBinder(fo.OwnerVote)
T.step_bodies([{k: np.float32(v) for k, v in body(0.0).items()}], np.float32(1.0))
T.bind([(np.int64(3), np.float32(0.0), np.float32(30.0))], np.float32(1.0))
try:
    json.dumps(T.body_list(np.float32(1.5)))
    ok = True
except TypeError as e:
    ok = str(e)
check("12 numpy 값 (float32·int64) 이 섞여도 몸 목록을 JSON 으로 씀", ok is True, ok)

print(f"\n{sum(results)}/{len(results)} 통과")
