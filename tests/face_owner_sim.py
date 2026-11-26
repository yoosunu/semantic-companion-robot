#!/usr/bin/env python3
"""face_owner_sim — 추적 한 명이 Jasper 인가 (nodes/perception/face_owner.py) 시험 (11-26). ROS 없음.
   python3 tests/face_owner_sim.py
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "nodes", "perception"))
import face_owner as fo  # noqa: E402

results = []


def check(name, ok, info=""):
    results.append(bool(ok))
    print(("통과 " if ok else "실패 ") + name + (f"  ({info})" if info != "" else ""))


def run(seq, **kw):
    v = fo.OwnerVote(**kw)
    states = [v.update(c, px, det)[0] for c, px, det in seq]
    return v, states


GOOD = (20.0, 0.9)          # 잘 보인 장 (크기, 검출 점수)
BAD = (8.0, 0.7)            # 작고 흐린 장

v, s = run([(0.5, *GOOD), (0.2, *GOOD), (0.45, *GOOD)])
check("1 3 장 중 2 장 ≥ 0.40 → jasper (세 번째 장에)", s == ["", "", "jasper"], s)
v, s = run([(0.5, *GOOD)])
check("2 한 장만 높음 → 아직 모름 (한 장 착각 0.73 % 를 믿지 않음)", s == [""], s)
v, s = run([(0.15, *BAD)] * 10)
check("3 낮은 점수라도 작고 흐린 장뿐 → 모름 (서 있는 나일 수 있음)", s[-1] == "", s[-1])
v, s = run([(0.15, *GOOD)] * 5)
check("4 잘 보인 장 5 장 전부 < 0.30 → stranger (다섯 번째에)", s == ["", "", "", "", "stranger"], s)
v, s = run([(0.15, *GOOD)] * 4 + [(0.33, *GOOD)])
check("5 잘 보인 5 장 중 하나라도 0.30 이상 → 아직 모름", s[-1] == "", s[-1])
v, s = run([(0.15, *GOOD)] * 5 + [(0.5, *GOOD), (0.5, *GOOD)])
check("6 stranger 였어도 2/3 가 넘으면 → jasper (각도가 나빴던 나)", s[-1] == "jasper", s)
v, s = run([(0.5, *GOOD)] * 3 + [(0.35, *BAD)] * 20)
check("7 jasper 뒤 뒤돌기·흐림 (잘 보인 장 없음) → jasper 유지", s[-1] == "jasper", s[-1])
v, s = run([(0.5, *GOOD)] * 3 + [(0.1, *GOOD)] * 5)
check("8 jasper 뒤 잘 보인 5 장 전부 안 맞음 (추적이 남으로 바뀜) → stranger", s[-1] == "stranger", s[-3:])
v, s = run([(0.5, *GOOD)] * 3 + [(0.1, *GOOD)] * 4 + [(0.45, *GOOD)])
check("9 jasper 뒤 4 장만 안 맞고 다시 맞음 → jasper 유지", s[-1] == "jasper", s[-2:])
v, s = run([(0.45, *BAD), (0.1, *BAD), (0.42, *BAD)])
check("10 작고 흐린 장이어도 2/3 ≥ 0.40 이면 jasper (높은 점수는 크기와 상관없이 증거)", s[-1] == "jasper", s)

print(f"\n{sum(results)}/{len(results)} 통과")
