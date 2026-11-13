"""executive_brain — 사령관의 판단 (ROS 없음, 순수 계산). executive 노드가 씀, tests/executive_sim.py 로 시험.

  설계: (프로젝트 문서) design_executive_v1_2026-11-14 + design_behavior_v2_2026-11-14.
  하는 일: 지금 모드를 정하고, 모드마다 **설정표(profile)** 를 내려보냄. 노드는 자기 칸만 따름.

  모드
    IDLE       자유 시간 (기본). 혼자이거나, 사람이 있어도 상호작용이 없음 → 30~60 s (랜덤) 조용하면 DOZE
    DOZE       꾸벅. 모양·횟수(nods)는 look 이 함. look 이 꾸벅을 다 하고 "asleep" 을 보내면 SLEEP (안 오면 90 s)
    COMPANION  같이 있기 (상호작용이 있었음). look 이 쳐다보기를 끝내고 "bored" 를 보낸 뒤 5 s 안에
               다시 상호작용이 없으면 IDLE. (bored 가 안 와도 마지막 상호작용 40 s 뒤 IDLE — 안전장치)
    SLEEP      잠. 얼굴 처리 끔. 자극(LiDAR 새로 들어옴·톡톡·나중에 소리·이름)으로만 깸
    TASK:<이름> 예약 작업 (v1 은 틀만). 요청 done:<이름> 이면 IDLE
  이벤트 (look 이 보냄): contact 눈맞춤 · approach 다가옴 · arrive 새 사람 · glance 두리번 · peekaboo 까꿍 ·
                       follow_gaze 시선 따라감 · nod 따라 끄덕  = 상호작용 / bored 쳐다보기 끝 / asleep 꾸벅 다 함
  자극 (잠 깨우기): (종류, 세기 1약·2중·3강, 방향 pan°|None)  — 동물 수면 참고 (설계 v2 4절)
    잠든 지 shallow (2분) 안 = 얕은 잠: 약한 자극에도 깸 / 그 뒤 = 깊은 잠: 강(3)만 깸, 중(2)은 뒤척임(stir), 약(1)은 무시
    DOZE 는 얕음: 어떤 자극이든 깸. 깨는 모양: 세기 2 이상 = 번쩍 + 자극 쪽으로, 1 = 천천히
  요청: sleep · wake · task:<이름> · done:<이름>

  시간: DUR 의 (최소, 최대) 에서 균등 랜덤 (11-14 Jasper: "네리 맘대로"). dur() 한 곳만 바꾸면 분포 교체.
"""
import random

DUR = {
    "doze_after": (30.0, 60.0),       # IDLE 에서 조용히 → 꾸벅 시작 (11-14 Jasper: 30~60)
    "companion_grace": (5.0, 5.0),    # look 이 쳐다보기 끝낸 뒤(bored) 다시 상호작용 기다림
    "companion_max": (40.0, 40.0),    # bored 가 안 와도 마지막 상호작용 뒤 이만큼이면 IDLE (look 쳐다보기 최대 30 s + 여유)
    "asleep_timeout": (90.0, 90.0),   # DOZE 에서 asleep 이 안 오면 이만큼 뒤 SLEEP
    "shallow": (120.0, 120.0),        # 잠든 뒤 얕은 잠
    "defer_max": (600.0, 600.0),      # 예약 작업: 같이 있는 중이면 최대 이만큼 미룸 (11-14 Jasper: 10분)
    "lidar_look": (10.0, 10.0),       # 자극으로 깼는데 얼굴이 없으면 이만큼 둘러보고 다시 꾸벅
}
NODS = 3                               # 꾸벅 3번째에 잠듦 (11-14 Jasper)


#            얼굴 Hz   YOLO Hz  바닥 Hz   고개          바퀴
PROFILE = {
    "SLEEP":     dict(face_hz=0.0, yolo_hz=0.0, floor_hz=0.0, head="sleep", wheels="stay"),
    "IDLE":      dict(face_hz=2.0, yolo_hz=0.0, floor_hz=0.0, head="free", wheels="stay"),
    "DOZE":      dict(face_hz=2.0, yolo_hz=0.0, floor_hz=0.0, head="doze", wheels="stay"),
    "COMPANION": dict(face_hz=5.0, yolo_hz=0.0, floor_hz=0.0, head="free", wheels="stay"),
    "TASK":      dict(face_hz=1.0, yolo_hz=0.0, floor_hz=0.0, head="free", wheels="task"),   # v1 자리만
}
INTERACT = {"contact", "approach", "arrive", "glance", "peekaboo", "follow_gaze", "nod", "flinch"}


class Executive:
    def __init__(self, now=0.0, rng=None):
        self.rng = rng or random.Random()
        self.mode, self.why, self.since = "IDLE", "start", now
        self.event_t = -1e9                # 마지막 상호작용
        self.grace_until = None            # COMPANION: bored 뒤 기다리는 끝
        self.doze_at = now + self.dur("doze_after")
        self.short_look = False            # 자극으로 깬 직후 (얼굴 없으면 짧게 둘러보고 다시 꾸벅)
        self.pending = None                # 미룬 예약 작업 (이름, 늦어도 이때까지)
        self.wake = None                   # 마지막 깨움 {"id","strength","dir"} — look 이 깨는 모양에 씀
        self.stir = 0                      # 뒤척임 횟수 (바뀌면 look 이 살짝 움직임)
        self.log = []                      # [(시각, 이전, 다음, 이유)]

    def dur(self, name):
        """시간 하나 (균등 랜덤). 나중에 로그정규·감정으로 바꿀 자리"""
        lo, hi = DUR[name]
        return self.rng.uniform(lo, hi)

    def go(self, now, mode, why):
        if mode == self.mode:
            return
        self.log.append((round(now, 1), self.mode, mode, why))
        self.mode, self.why, self.since = mode, why, now
        self.grace_until = None
        if mode == "IDLE":
            self.doze_at = now + (self.dur("lidar_look") if self.short_look else self.dur("doze_after"))

    def wake_up(self, now, why, strength, direction):
        self.wake = {"id": (self.wake or {}).get("id", 0) + 1, "strength": "strong" if strength >= 2 else "weak",
                     "dir": direction}
        self.short_look = True
        self.go(now, "IDLE", why)

    def profile(self):
        base = "TASK" if self.mode.startswith("TASK:") else self.mode
        return dict(mode=self.mode, why=self.why, nods=NODS, wake=self.wake, stir=self.stir, **PROFILE[base])

    def tick(self, now, events=(), people=False, stimuli=(), requests=()):
        """events: [(종류, 트랙)] look 이 보낸 것 / people: 지금 얼굴이 보이나
           stimuli: [(종류, 세기 1~3, 방향 pan°|None)] 깨우기 자극 / requests: ["sleep" | "wake" | "task:이름" | "done:이름"]"""
        for r in requests:                                   # 1. 요청 (사람이 시킨 것) 이 먼저
            if r == "sleep":
                self.go(now, "SLEEP", "request")
            elif r == "wake" and self.mode in ("SLEEP", "DOZE"):
                self.wake_up(now, "request", 2, None)
            elif r.startswith("task:"):
                if self.mode == "COMPANION":
                    self.pending = (r[5:], now + self.dur("defer_max"))   # 같이 있는 중 → 미룸
                else:
                    self.go(now, "TASK:" + r[5:], "task")
            elif r.startswith("done:") and self.mode == "TASK:" + r[5:]:
                self.go(now, "IDLE", "task done")

        if self.mode == "SLEEP":                             # 2. 잠: 자극으로만 깸 (세기 vs 깊이)
            shallow = now - self.since < self.dur("shallow")
            for kind, s, d in sorted(stimuli, key=lambda x: -x[1]):
                if s >= 3 or (shallow and s >= 1):
                    self.wake_up(now, f"{kind} ({'얕은' if shallow else '깊은'} 잠)", s, d)
                    break
                if s == 2:
                    self.stir += 1                           # 깊은 잠 + 중간 자극 = 뒤척임
                    self.log.append((round(now, 1), "SLEEP", "SLEEP", f"stir: {kind}"))
                    break
            return self.profile()

        if self.mode == "DOZE" and stimuli:                  # 3. 꾸벅은 얕음: 어떤 자극이든 깸
            kind, s, d = max(stimuli, key=lambda x: x[1])
            self.wake_up(now, kind + " (꾸벅)", max(s, 2), d)

        names = [e[0] for e in events]
        ev = [e for e in events if e[0] in INTERACT]
        if ev and not self.mode.startswith("TASK:"):         # 4. 상호작용 → 같이 있기 (작업 중이면 작업 계속)
            self.event_t = now
            self.short_look = False
            self.grace_until = None
            if self.mode == "DOZE":
                self.wake = {"id": (self.wake or {}).get("id", 0) + 1, "strength": "strong", "dir": None}
            self.go(now, "COMPANION", f"{ev[0][0]} #{ev[0][1]}")
        if people and self.short_look:                       # 얼굴을 봤으면 자극 깸은 진짜였음 → 보통 규칙 (30~60 s)
            self.short_look = False
            if self.mode == "IDLE":
                self.doze_at = now + self.dur("doze_after")

        if self.mode == "COMPANION":
            if "bored" in names and not ev:
                self.grace_until = now + self.dur("companion_grace")
            end = (self.grace_until is not None and now >= self.grace_until) or \
                  now - self.event_t > self.dur("companion_max")
            if self.pending and now >= self.pending[1]:      # 5. 미룬 작업: 늦어도 이때는 감
                self.go(now, "TASK:" + self.pending[0], "task (deferred max)")
                self.pending = None
            elif end:
                if self.pending:
                    self.go(now, "TASK:" + self.pending[0], "task (after companion)")
                    self.pending = None
                else:
                    self.go(now, "IDLE", "quiet")
        elif self.mode == "IDLE":
            if now >= self.doze_at:
                self.go(now, "DOZE", "woke: nobody" if self.short_look else "quiet")
                self.short_look = False
        elif self.mode == "DOZE":
            if "asleep" in names or now - self.since > self.dur("asleep_timeout"):
                self.go(now, "SLEEP", "asleep" if "asleep" in names else "doze timeout")
        return self.profile()
