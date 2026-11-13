"""executive_brain — 사령관의 판단 (ROS 없음, 순수 계산). executive 노드가 씀, tests/executive_sim.py 로 시험.

  설계: docs (프로젝트) design_executive_v1_2026-11-14 — "얇은 상위 계층 먼저". 지금 있는 모듈만 다룸.
  하는 일: 지금 모드를 정하고, 모드마다 **설정표(profile)** 를 내려보냄. 노드는 자기 칸만 따름.

  모드 (v1)
    IDLE       자유 시간 (기본). 혼자이거나, 사람이 있어도 상호작용이 없음
    DOZE       꾸벅 (IDLE 에서 조용히 doze_after 지나면). 표현은 look 이 함
    COMPANION  같이 있기 (상호작용 이벤트가 있었음). companion_keep 동안 이벤트 없으면 IDLE
    SLEEP      잠 (DOZE 로 sleep_after 지나면 / 요청). 얼굴 처리 끔. LiDAR "새로 들어옴" 으로만 깸
    TASK:<이름> 예약 작업 (v1 은 틀만 — 지도·순찰이 생기면 여기로). 요청 done:<이름> 이면 IDLE
  이벤트 = look 이 보낸 상호작용 (contact 눈맞춤 · approach 다가옴 · arrive 새 사람 · glance 두리번)
  요청 = 사람/다른 노드가 보낸 글자: sleep · wake · task:<이름> · done:<이름>

  시간들은 지금은 상수 (11-14 Jasper: "신뢰와 상관없는 부분이라 나중에 랜덤으로 갈아끼울 수 있게") → dur() 한 곳만 바꾸면 됨.
"""

DUR = {
    "doze_after": 60.0,        # IDLE 에서 이벤트 없이 → 꾸벅
    "sleep_after": 180.0,      # 꾸벅에서 → 잠
    "companion_keep": 120.0,   # COMPANION 에서 이벤트 없이 → IDLE (11-14 Jasper: 2분)
    "defer_max": 600.0,        # 예약 작업: 같이 있는 중이면 최대 이만큼 미룸 (11-14 Jasper: 10분)
    "lidar_look": 10.0,        # LiDAR 로 깼는데 얼굴이 없으면 이만큼 둘러보고 다시 꾸벅
}


def dur(name):
    """시간 하나. 나중에 랜덤(예: 로그정규)으로 바꿀 자리"""
    return DUR[name]


#            얼굴 Hz   YOLO Hz  바닥 Hz   고개          바퀴
PROFILE = {
    "SLEEP":     dict(face_hz=0.0, yolo_hz=0.0, floor_hz=0.0, head="sleep", wheels="stay"),
    "IDLE":      dict(face_hz=2.0, yolo_hz=0.0, floor_hz=0.0, head="free", wheels="stay"),
    "DOZE":      dict(face_hz=2.0, yolo_hz=0.0, floor_hz=0.0, head="doze", wheels="stay"),
    "COMPANION": dict(face_hz=5.0, yolo_hz=0.0, floor_hz=0.0, head="free", wheels="stay"),
    "TASK":      dict(face_hz=1.0, yolo_hz=0.0, floor_hz=0.0, head="free", wheels="task"),   # v1 자리만
}
INTERACT = {"contact", "approach", "arrive", "glance"}


class Executive:
    def __init__(self, now=0.0):
        self.mode, self.why, self.since = "IDLE", "start", now
        self.quiet_t = now                 # IDLE·DOZE 조용함 시작 (이벤트마다 다시)
        self.event_t = -1e9                # 마지막 상호작용
        self.short_look = False            # LiDAR 로 깬 직후 (얼굴 없으면 짧게 둘러보고 다시 꾸벅)
        self.pending = None                # 미룬 예약 작업 (이름, 늦어도 이때까지)
        self.log = []                      # [(시각, 이전, 다음, 이유)]

    def go(self, now, mode, why):
        if mode == self.mode:
            return
        self.log.append((round(now, 1), self.mode, mode, why))
        self.mode, self.why, self.since = mode, why, now
        if mode in ("IDLE", "DOZE"):
            self.quiet_t = now

    def profile(self):
        base = "TASK" if self.mode.startswith("TASK:") else self.mode
        return dict(mode=self.mode, why=self.why, **PROFILE[base])

    def tick(self, now, events=(), people=False, lidar_in=False, requests=()):
        """events: [(종류, 트랙)] 이번에 온 상호작용 / people: 지금 얼굴이 보이나 / lidar_in: 잠잘 때 LiDAR 새로 들어옴
           requests: ["sleep" | "wake" | "task:이름" | "done:이름"]"""
        for r in requests:                                   # 1. 요청 (사람이 시킨 것) 이 먼저
            if r == "sleep":
                self.go(now, "SLEEP", "request")
            elif r == "wake" and self.mode in ("SLEEP", "DOZE"):
                self.go(now, "IDLE", "request")
            elif r.startswith("task:"):
                if self.mode == "COMPANION":
                    self.pending = (r[5:], now + dur("defer_max"))   # 같이 있는 중 → 미룸
                else:
                    self.go(now, "TASK:" + r[5:], "task")
            elif r.startswith("done:") and self.mode == "TASK:" + r[5:]:
                self.go(now, "IDLE", "task done")

        if self.mode == "SLEEP":                             # 2. 잠: LiDAR 로만 깸
            if lidar_in:
                self.short_look = True
                self.go(now, "IDLE", "lidar")
            return self.profile()

        ev = [e for e in events if e[0] in INTERACT]
        if ev and not self.mode.startswith("TASK:"):         # 3. 상호작용 → 같이 있기 (작업 중이면 작업 계속)
            self.event_t = now
            self.short_look = False
            self.go(now, "COMPANION", f"{ev[0][0]} #{ev[0][1]}")
        if people:
            self.short_look = False                          # 얼굴을 봤으면 LiDAR 깸은 진짜였음 → 보통 규칙

        if self.mode == "COMPANION":
            if self.pending and now >= self.pending[1]:      # 4. 미룬 작업: 늦어도 이때는 감
                self.go(now, "TASK:" + self.pending[0], "task (deferred max)")
                self.pending = None
            elif now - self.event_t > dur("companion_keep"):
                if self.pending:
                    self.go(now, "TASK:" + self.pending[0], "task (after companion)")
                    self.pending = None
                else:
                    self.go(now, "IDLE", "quiet")
        elif self.mode == "IDLE":
            limit = dur("lidar_look") if self.short_look else dur("doze_after")
            if now - self.quiet_t > limit:
                self.go(now, "DOZE", "lidar: nobody" if self.short_look else "quiet")
                self.short_look = False
        elif self.mode == "DOZE":
            if now - self.since > dur("sleep_after"):
                self.go(now, "SLEEP", "quiet")
        return self.profile()
