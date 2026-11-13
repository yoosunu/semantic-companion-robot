"""lidar_motion — LiDAR 로 "누가 새로 가까이 옴" 감지 (잠 깨우기용, 순수 계산). executive 가 SLEEP 일 때 씀.
   11-12 look_brain 에서 떼어 냄 (11-14 사령관 v1: 잠·깨우기는 executive 담당).
"""

import math


class Motion:
    """LiDAR 로 "누가 새로 가까이 옴" 감지 (잠잘 때 깨우기). 로봇이 서 있다는 가정.

       11-12 실측: 잠들자마자 2 s 만에 깸 반복 (꾸벅 30 s → 잠 → 곧 깸 → 두리번 20 s → … 52 s 주기).
       원인: Jasper 가 로봇 바로 옆(1.5 m 안)에 앉아 있음 → 몸을 조금만 움직여도 "가까운 움직임" 으로 깸.
       Jasper 가 지루해서 잠든 건데 Jasper 가 있다고 깨면 안 됨 → 규칙을 "움직임" 에서 "새로 들어옴" 으로:
         1. 잠든 뒤 learn_s (3 s) 동안 빔마다 최소·최대 거리를 배움 (이미 있는 사람·물체, 원래 흔들리는 폭)
         2. 그때 near_m 밖(비어 있던 곳)이던 빔에 near_m 안쪽으로 무언가 들어오면 "들어옴"
            (이미 안에 있던 사람이 꼼지락거리는 건 안 셈)
         3. 0.3 m 안쪽 빔은 무시 (로봇 몸)
         4. 들어온 빔 n 개 이상이 3 스캔 연속이면 깸
       last = 마지막 스캔에서 들어온 빔 수 (로그용)"""

    def __init__(self, near_m=1.5, n=6, learn_s=3.0):
        self.near, self.n, self.learn = near_m, n, learn_s
        self.t0, self.lo, self.hi = None, None, None
        self.hits, self.last = 0, 0
        self.last_idx = None                                 # 마지막으로 들어온 빔들의 가운데 번호

    def step(self, now, ranges):
        r = [x if (x == x and 0.3 < x < 12.0) else None for x in ranges]   # nan·inf·0·몸 → 무시
        if self.t0 is None or self.lo is None or len(self.lo) != len(r):
            self.t0 = now
            self.lo, self.hi = [99.0] * len(r), [-1.0] * len(r)
        if now - self.t0 < self.learn:                        # 지금 있는 것들 배우기
            for i, x in enumerate(r):
                if x is not None:
                    self.lo[i], self.hi[i] = min(self.lo[i], x), max(self.hi[i], x)
            return False
        idx = [i for i, x in enumerate(r)
               if x is not None and x < self.near and self.lo[i] > self.near + 0.2]   # 비어 있던 곳에 들어옴
        came = len(idx)
        self.last = came
        if idx:                                              # 들어온 쪽 (빔 번호 평균, 원형) — 노드가 각도로 바꿈
            n = len(r)
            sx = sum(math.cos(2 * math.pi * i / n) for i in idx)
            sy = sum(math.sin(2 * math.pi * i / n) for i in idx)
            self.last_idx = (math.atan2(sy, sx) / (2 * math.pi) * n) % n
        self.hits = self.hits + 1 if came >= self.n else 0
        return self.hits >= 3
