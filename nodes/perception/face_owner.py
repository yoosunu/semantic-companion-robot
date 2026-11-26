"""face_owner — 추적 한 명이 Jasper 인가 (11-26). ROS 없음 → tests/face_owner_sim.py, tools/lab/face_eval.py replay.

  목표 (Jasper 11-26): 여러 사람 사이에서도 나를 알아봄. "누구냐" 가 아니라 "Jasper 냐 아니냐".
    부르면 Jasper 추적을 따라가고, 다른 사람과 상호작용해도 그게 내가 아닌 걸 알아야 함.

  상태 (추적마다)
    "jasper"   최근 n 장 중 k 장이 문턱 thr 이상 (11-26 실측: 0.40 2/3 → 공개 사진 1219 묶음 중 착각 0, 나는 2~3 장 만에)
    "stranger" 잘 보인 장 (얼굴 ≥ min_px, 검출 ≥ min_det) 최근 not_k 장이 전부 not_thr 미만
               — 점수가 낮다고 바로 '아님' 이 아님: 서 있는 나는 옛 등록으로 한 장 정답 19% 까지 떨어졌음.
                 잘 보인 장이 여러 장 쌓였는데 다 안 맞을 때만.
    ""         아직 모름 (얼굴을 제대로 못 봄)
  바뀜
    모름 → jasper / stranger, stranger → jasper (jasper 규칙이 더 엄격 = 착각 0 쪽)
    jasper → stranger: 잘 보인 장 not_k 장이 전부 안 맞으면 (추적이 다른 사람으로 바뀐 경우 — 엉뚱한 사람을 계속 따라가지 않게)
"""
import collections


class OwnerVote:
    def __init__(self, thr=0.40, k=2, n=3, not_thr=0.30, not_k=5, min_px=10.0, min_det=0.80):
        self.thr, self.k, self.n = thr, k, n
        self.not_thr, self.not_k, self.min_px, self.min_det = not_thr, not_k, min_px, min_det
        self.hits = collections.deque(maxlen=n)          # 최근 n 장: 문턱 넘었나
        self.good = collections.deque(maxlen=not_k)      # 최근 잘 보인 장의 유사도
        self.state, self.best, self.frames = "", -1.0, 0

    def update(self, cos, px, det):
        """한 장 → (상태, 바뀌었나)"""
        self.frames += 1
        self.best = max(self.best, cos)
        self.hits.append(cos >= self.thr)
        if px >= self.min_px and det >= self.min_det:
            self.good.append(cos)
        old = self.state
        if sum(self.hits) >= self.k:
            self.state = "jasper"
        elif len(self.good) == self.not_k and max(self.good) < self.not_thr:
            self.state = "stranger"
            self.hits.clear()
        return self.state, self.state != old

    def votes(self):
        return int(sum(self.hits))
