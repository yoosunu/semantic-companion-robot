"""look_brain — 고개의 "마음" (ROS 없음, 순수 계산). look_at_person 노드가 씀, tests/look_brain_sim.py 로 시험.

  원칙 (11-12 Jasper): 사람이 있다고 계속 쳐다보면 감시 카메라. **쳐다볼 이유가 있을 때만** 쳐다본다.

  모드
    scan    아무도 없음 → 4~7 s 마다 좌우 한 곳 두리번
    attend  사람을 쳐다봄 (가운데 ±10°/±8° 밖이면 한 번에 확). 5~30 s (매번 랜덤, 11-14 Jasper) 지나면 → idle + "bored" 알림
            눈맞춤 다듬기: 안쪽이어도 1.2 s 가만히 있은 뒤 평균 오차가 3° 넘으면 작게 한 번 더 (사람 눈의 "보정 도약").
            11-12 Jasper: "끄덕일 때·가까울 때 눈을 살짝 비껴 봄 → 눈맞춤 느낌이 안 남". 계속 따라가는 게 아니라 한 번씩 → 덜덜 없음
    idle    딴짓: 사람은 곁눈(화면 안)에 두고 15~30° 옆을 봄, 5~12 s 마다 바꿈. 트리거가 오면 → attend
            사람을 놓쳐도 마지막 본 곳 기준 (11-12: 놓친 뒤 지금 고개 기준으로 또 옆을 봐서 −58° → −96° 로 멀어짐)
    doze    꾸벅 (executive 가 head=doze, nods=3). 꾸벅 사이에도 아주 느리게 가라앉음 (멈춰서 아래 보는 것과 구별)
            꾸벅 한 번 (11-14 Jasper: "툭, 스르르륵....툭 깸" / "스르르륵....툭...깸") — 매번 A·B, 각도·시간 랜덤
              A: 툭(작게) → 스르르륵(아주 느리게 8~12°) → 툭(빠르게 6~10°) → 멈칫 → 깸(원래보다 살짝 위로, 빠르게) → 천천히 제자리
              B: 스르르륵 → 툭 → (조금 길게) 멈칫 → 깸
            nods 번째는 안 깸: 스르르륵 → 툭 → 천천히 끝까지 숙임 → "asleep" 알림 (executive 가 SLEEP)
    sleep   잠 (executive 가 head=sleep). 고개 끝까지 숙임. 뒤척임(stir) = 살짝 움직였다 다시
            깨기 (head 가 free 로): wake.strength strong = 번쩍 + 자극 쪽(dir)으로 / weak = 천천히 들고 두리번
  11-14 사령관 v1: **언제** 졸고 자고 깨는지는 executive (nodes/behavior/executive_brain.py) 가 정함.
    여기는 **모양**만 (set_head). 트리거는 바로 반응하고 events 에 쌓아서 executive 에 알림 (/neri/event).
    졸 때도 트리거를 보면 알리기만 함 → executive 가 COMPANION 으로 → head=free → 번쩍.

  다시 쳐다볼 이유 (트리거) — 이게 있어야 attend 로
    contact   사람이 나를 쳐다보기 시작 (2 s 넘게 딴 데(|yaw|>35°) 보던 사람이 20° 안으로)
    glance    사람이 두리번 (1 s 안에 yaw 25° 넘게 바뀜) → 짧게 3 s 만 봄. ※ 기본 끔 (아래)
    approach  다가옴 — 로봇이 서 있을 때만 (로봇이 다가가도 얼굴은 커지니까. 11-12 Jasper)
              (최근 0.5 s 평균 얼굴이 1 s 전보다 1.3 배, 20 px 이상)
    arrive    새 사람 (10 s 동안 아무도 없다가) → 쳐다보고 끄덕 인사
  행동 클립 (11-12 목록 ①②④⑤, 얼굴 CLIP_MIN_PX 이상, 쿨다운, 하나씩, 꾸벅·잠 중엔 안 함)
    ① 까꿍 (peekaboo)     쳐다보던 얼굴이 같은 자리(10°)에서 0.5~3 s 사라졌다 다시 → 놀람(젖힘) → 끄덕 2번
    ② 시선 따라가기        눈맞춤하던 사람이 고개를 30° 넘게 돌려 1 s 유지 → 그쪽(사람 + 30°)을 1.5 s 힐끗 → 다시 사람
    ④ 움찔 (flinch)       아주 빠른 다가옴 (1.6배) → 뒤로 젖힘 → 올려다봄  (나중에 "빵!" 개인기로)
    ⑤ 따라 끄덕 (nod)      눈맞춤 중 사람이 끄덕 → 0.3 s 뒤 같이 끄덕. pitch 가 거칠어 기본 끔 (NOD_ON)
  단순히 "있음" 은 트리거가 아님 → 사람이 컴퓨터만 보고 있으면 로봇은 딴짓하다 졸게 됨.
  이미 쳐다보는 중(attend)엔 contact·glance 가 시간을 늘리지 않음 → 10~30 s 지나면 꼭 딴짓 (다시 보려면 고개 돌렸다 다시 봐야 함)

  yaw 믿을 조건 (11-12 실측: 얼굴 14~25 px 에선 yaw 가 −8 → −52 → −90 으로 1 s 마다 튐 → glance 가 계속 터져 영원히 쳐다봄)
    얼굴 YAW_MIN_PX 이상일 때만, 최근 0.6 s 중앙값으로. glance 는 그래도 불안해서 기본 끔 (GLANCE_ON)
  LED (11-12 실측: 8 도 눈부신 조명) → 노드 파라미터 led (기본 false) 로 안 보냄. 나중에 밤 조명 등으로

  출력 동작: ("move", 팬, 틸트, SPD, ACC)  SPD/ACC 0 = 서보 최고 속도 / ("led", 0~255)
  알림 (events): contact approach arrive glance peekaboo follow_gaze nod (상호작용) / bored / asleep
"""
import collections
import random

ZONE_X, ZONE_Y = 10.0, 8.0           # attend: 이 안이면 가만히
SETTLE = 0.6                         # 빠른 고개 이동 뒤 기다림
ATTEND_S = (5.0, 30.0)               # 한 번 쳐다보면 이만큼 (랜덤, 11-14 Jasper: 5~30)
GLANCE_S = 3.0
IDLE_EVERY = (5.0, 12.0)             # 딴짓 시선 바꾸는 간격
IDLE_OFF = (15.0, 30.0)              # 딴짓: 사람에서 이만큼 옆 (화각 ±60° 안. 11-12: 45° 면 어안 가장자리라 먼 얼굴 놓침)
FINE, FINE_WAIT = 3.0, 1.2           # 눈맞춤 다듬기: 오차 3° 넘고, 마지막 이동 뒤 1.2 s
SCAN_AFTER = 5.0
SCAN_EVERY = (6.0, 14.0)            # 11-12 Jasper: 4~7 s 는 너무 두리번거림
SCAN_PANS = (-70.0, -35.0, 0.0, 35.0, 70.0)
LOST_S = 2.0
CONTACT_IN, CONTACT_OUT = 20.0, 35.0
CONTACT_AWAY_S = 2.0                 # 이만큼 딴 데 봐야 다음 contact 인정
GLANCE_YAW = 25.0
GLANCE_ON = False
YAW_MIN_PX = 20.0                    # 이보다 작은 얼굴은 yaw 안 믿음
YAW_MED_S = 0.6
APPROACH_RATIO, APPROACH_WIN = 1.3, 1.5    # 평균끼리 비교라 1.3 (1 s 전보다 30% 커짐)
APPROACH_MIN_PX = 20.0               # 다가와서 이만큼은 커져야 (11-12: 먼 얼굴 15↔21 px 흔들림이 "다가옴" 으로 잡힘)
ARRIVE_GAP = 10.0
LED_AWAKE, LED_DOZE, LED_FLASH = 40, 8, 255
SLOW = (150, 5)                      # 졸기 시작 느린 SPD, ACC (ST3215: SPD 걸음/s, 4096 걸음 = 한 바퀴 → 150 ≈ 13°/s)
STEPS_PER_DEG = 4096 / 360.0         # ST3215 SPD 단위 (걸음/s)
NOD_GAP = (1.0, 3.0)                 # 꾸벅 사이 (11-14 실물: 10~20 s 면 동작이 띄엄띄엄 → 조는 걸로 안 보임. 깨자마자 다시 스르르)
CLIP_MIN_PX = 25.0                   # 행동 클립은 이만큼 큰 얼굴에서만 (yaw·pitch 믿을 크기)
COOL = {"peekaboo": 10.0, "follow_gaze": 20.0, "flinch": 10.0, "nod": 8.0}
PEEK_GAP = (0.5, 3.0)
FOLLOW_YAW, FOLLOW_HOLD, FOLLOW_OFF = 30.0, 1.0, 30.0
FLINCH_RATIO = 1.6
NOD_ON = False                       # ⑤ 따라 끄덕: pitch 가 거칠어 실물 보고 켬
TILT_HOME, TILT_DOZE, TILT_SLEEP = 10.0, -5.0, -30.0


class Brain:
    def __init__(self, rng=None):
        self.rng = rng or random.Random()
        self.pan, self.tilt = 0.0, TILT_HOME
        self.mode, self.reason = "scan", "start"
        self.busy_until = 0.0
        self.clip = []                       # 예약 동작 [(시각, 동작)]
        self.target = None                   # 쳐다보는 트랙 id
        self.until = 0.0                     # attend/idle/scan 다음 바꿀 시각
        self.head = "free"                   # executive 가 정한 고개 상태 (free / doze / sleep)
        self.events = []                     # executive 에 알릴 트리거 [(이름, 트랙)] — 노드가 꺼내 감
        self.seen_t = -1e9                   # 마지막으로 누군가 보인 시각
        self.out_n = 0                       # attend: 가운데 밖으로 연속 몇 번
        self.errs = []                       # attend: 최근 오차 (눈맞춤 다듬기)
        self.moved_t = 0.0
        self.tpan, self.ttilt = 0.0, TILT_HOME   # 쳐다보던 사람을 마지막으로 본 방향
        self.mem = {}                        # 트랙별 {"yaw": deque[(t,yaw)], "px": deque[(t,px)], "contact": bool}
        self.started = False
        self.last_trig = None                # (시각, 이름, 트랙) 로그용
        self.nods_left, self.base_tilt = 0, TILT_DOZE   # 꾸벅: 남은 횟수, 지금 졸린 자세 높이
        self.wake_id, self.stir_id = 0, 0
        self.cool = {}                       # 클립별 마지막 시각
        self.peek = None                     # 까꿍: (마지막으로 본 시각, 팬, 틸트)
        self.peek_back = None                # 까꿍: 사라졌다 다시 나타남 (시각, 사람)
        self.peek_hidden = False             # 까꿍: 정말 가려졌었나
        self.attend_t = -1e9                 # attend 시작 (막 쳐다보기 시작한 3 s 는 까꿍 안 함 — 깨자마자 가짜 까꿍)
        self.approach0 = None                # 움찔: (다가옴 시각, 다가오기 전 얼굴 px, 트랙)
        self.follow_done = None              # 시선 따라가기: 이번 고개 돌림에 이미 했나 (away_t)

    # ---------- 작은 도구 ----------
    def move(self, out, now, pan, tilt, spd=0, acc=0, settle=SETTLE):
        pan = (pan + 180.0) % 360.0 - 180.0
        self.pan, self.tilt = pan, max(-30.0, min(90.0, tilt))
        out.append(("move", self.pan, self.tilt, spd, acc))
        self.busy_until, self.moved_t, self.errs = now + settle, now, []

    def play(self, now, steps):
        """steps: [(몇 초 뒤, 동작)] 동작 = ("move", pan, tilt, spd, acc, settle) / ("led", v)"""
        self.clip = sorted([(now + dt, a) for dt, a in steps], key=lambda x: x[0])

    def nod(self, now, pan, tilt, times=1):
        steps, t = [], 0.0
        for _ in range(times):
            steps += [(t + 0.5, ("move", pan, tilt - 8.0, 0, 0, 0.3)), (t + 0.85, ("move", pan, tilt, 0, 0, 0.3))]
            t += 0.7
        steps += [(0.4, ("led", LED_FLASH)), (t + 0.6, ("led", LED_AWAKE))]
        self.play(now, steps)

    def set_mode(self, mode, reason, now):
        if mode != self.mode or reason != self.reason:
            self.mode, self.reason = mode, reason

    # ---------- 트리거 감지 ----------
    def triggers(self, now, people, moving=False):
        """사람마다 이번에 생긴 트리거 → [(이름, 트랙)]"""
        found = []
        for p in people:
            if p["unseen"] > 0:
                continue
            m = self.mem.setdefault(p["id"], {"yaw": collections.deque(maxlen=30), "px": collections.deque(maxlen=30),
                                              "smooth": collections.deque(maxlen=30), "pitch": collections.deque(maxlen=30),
                                              "contact": None, "away_t": now})
            m["pitch"].append((now, p.get("pitch", 0.0)))
            m["px"].append((now, p["px"]))
            cur = [x for t, x in m["px"] if now - t <= 0.5]          # 최근 0.5 s 평균 vs 1 s 전쯤 평균 (흔들림은 평균에서 상쇄)
            old = [x for t, x in m["px"] if 0.8 <= now - t <= APPROACH_WIN]
            if moving:
                m["px"].clear()                              # 로봇이 움직이는 중: 크기 변화는 우리 탓
            elif cur and old and sum(cur) / len(cur) >= max(APPROACH_MIN_PX, APPROACH_RATIO * sum(old) / len(old)):
                found.append(("approach", dict(p, base=sum(old) / len(old))))
                m["px"].clear()
            if p["px"] < YAW_MIN_PX:
                continue                                     # 작은 얼굴: yaw 못 믿음
            m["yaw"].append((now, p["yaw"]))
            recent = sorted(y for t, y in m["yaw"] if now - t <= YAW_MED_S)
            yaw = recent[len(recent) // 2]                  # 중앙값 (튀는 값 하나는 무시)
            m["smooth"].append((now, yaw))
            if m["contact"] is None:
                m["contact"] = abs(yaw) <= CONTACT_IN        # 첫 관측은 기준만
            elif not m["contact"] and abs(yaw) <= CONTACT_IN:
                m["contact"] = True
                if now - m["away_t"] >= CONTACT_AWAY_S:
                    found.append(("contact", p))
            elif m["contact"] and abs(yaw) >= CONTACT_OUT:
                m["contact"], m["away_t"] = False, now
            elif not m["contact"] and abs(yaw) < CONTACT_OUT and abs(yaw) > CONTACT_IN:
                pass
            if GLANCE_ON:
                old = [y for t, y in m["smooth"] if now - t <= 1.0]
                if old and max(old) - min(old) >= GLANCE_YAW:
                    found.append(("glance", p))
                    m["smooth"].clear()
        ids = {p["id"] for p in people}
        for k in [k for k in self.mem if k not in ids]:
            del self.mem[k]
        return found

    # ---------- 한 번 (0.1 s 마다) ----------
    def set_head(self, head, now, nods=3, wake=None, stir=0):
        """executive 설정표의 head·nods·wake·stir 칸. 바뀔 때만 동작 → 이번에 보낼 동작 목록"""
        out = []
        if stir != self.stir_id:
            self.stir_id = stir
            if self.mode == "sleep":                         # 뒤척임: 살짝 움직였다 다시 잠
                pn = self.pan + self.rng.choice((-1, 1)) * self.rng.uniform(4.0, 8.0)
                self.play(now, [(0.0, ("move", pn, TILT_SLEEP + 4.0, *SLOW, 1.5)),
                                (2.0, ("move", self.pan, TILT_SLEEP, *SLOW, 1.5))])
        if head == self.head:
            return out
        prev, self.head = self.head, head
        if head == "doze":
            self.doze(out, now, "executive", nods)
        elif head == "sleep":
            self.set_mode("sleep", "executive", now)
            self.clip = []
            if self.tilt > TILT_SLEEP + 1:
                self.move(out, now, self.pan, TILT_SLEEP, *SLOW, settle=4.0)
            out.append(("led", 0))
        elif prev in ("doze", "sleep"):
            w = wake or {}
            if w.get("id") != self.wake_id:
                self.wake_id = w.get("id")
            self.wake(out, now, "executive", strength=w.get("strength", "strong"), direction=w.get("dir"))
        return out

    def tick(self, now, people, moving=False, age=0.0):
        """people: [{"id","pan","tilt","px","yaw","pitch","unseen"}] (/people 최신), moving: 로봇 바퀴가 움직이는 중,
           age: /people 이 마지막으로 온 지 몇 초 (보기가 멈춰 있으면 "안 보임" 을 믿지 않음)"""
        out = []
        if not self.started:
            self.started = True
            if self.head == "free":                          # 시작부터 졸거나 자라고 하면 고개 안 듦
                out.append(("led", LED_AWAKE))
                self.move(out, now, 0.0, TILT_HOME)
        while self.clip and now >= self.clip[0][0]:
            a = self.clip.pop(0)[1]
            if a[0] == "move":
                self.move(out, now, a[1], a[2], a[3], a[4], a[5])
            elif a[0] == "event":
                self.events.append((a[1], self.target if self.target is not None else -1))
            else:
                out.append(a)

        if self.mode == "sleep":                            # 잠: 아무것도 안 봄 (깨우는 건 executive)
            return out

        visible = [p for p in people if p["unseen"] <= 0.5]
        if visible:
            if now - self.seen_t > ARRIVE_GAP:
                trig = [("arrive", max(visible, key=lambda p: p["px"]))]
            else:
                trig = []
            self.seen_t = now
            trig += self.triggers(now, visible, moving)
        else:
            trig = []

        if trig:
            self.events += [(name, p["id"]) for name, p in trig]   # executive 에 알림 (꾸벅 중이어도)
        if self.mode == "doze":
            if not self.clip and now > self.until and self.nods_left > 0:
                self.nods_left -= 1
                end = self.nod_off(now, last=self.nods_left == 0)
                self.until = now + end + self.rng.uniform(*NOD_GAP)
            return out

        if trig:
            name, p = trig[0]
            self.last_trig = (round(now, 1), name, p["id"])
            if self.clip and name != "arrive":
                pass                                         # 인사 중이면 끝까지
            elif self.mode == "attend" and name in ("contact", "glance"):
                pass                                         # 이미 쳐다보는 중: 시간 안 늘림
            else:
                self.attend(out, now, name, p)

        if self.mode == "attend" and self.peek:             # ① 까꿍 추적은 클립·이동 중에도 계속 (시간 간격이 정확하게)
            near = [p for p in people if abs(p["pan"] - self.peek[1]) < 10 and abs(p["tilt"] - self.peek[2]) < 10]
            fresh = [p for p in near if p["unseen"] <= 0.1]
            if age <= 0.6 and not any(p["unseen"] <= 0.6 for p in near):
                self.peek_hidden = True                      # 보기가 지금 돌고 있는데 그 자리에 없음 = 진짜 가림                      # 진짜로 안 보임 (11-14: 얼굴 2 Hz 면 검출 사이 0.5 s 가 "가림" 으로 잡혀 가짜 까꿍)
            if fresh:
                p = min(fresh, key=lambda q: abs(q["pan"] - self.peek[1]))
                if self.peek_hidden and PEEK_GAP[0] <= now - self.peek[0] <= PEEK_GAP[1] and now - self.attend_t > 3.0:
                    self.peek_back = (now, p)                # 사라졌다 다시 나타남 → clips() 에서
                self.peek, self.peek_hidden = (now, p["pan"], p["tilt"]), False
        if self.clip or now < self.busy_until:
            return out
        tgt = next((p for p in people if p["id"] == self.target), None)
        if self.mode in ("attend", "idle") and (tgt is None or tgt["unseen"] > LOST_S):
            alt = max(visible, key=lambda p: p["px"]) if visible else None
            if alt:
                self.target = alt["id"]
                tgt = alt
            elif now - self.seen_t > SCAN_AFTER:
                self.set_mode("scan", "nobody", now)
                self.until = 0.0
        if tgt and tgt["unseen"] <= 0.1:
            self.tpan, self.ttilt = tgt["pan"], tgt["tilt"]
        if self.mode == "attend":
            if now > self.until:
                self.set_mode("idle", "bored", now)
                self.events.append(("bored", self.target if self.target is not None else -1))
                self.until = 0.0
            elif self.clips(out, now, people, tgt):
                pass                                         # 클립이 고개를 맡음
            elif tgt and tgt["unseen"] <= 0.1:
                dx, dy = tgt["pan"] - self.pan, tgt["tilt"] - self.tilt
                if abs(dx) > ZONE_X or abs(dy) > ZONE_Y:
                    self.out_n += 1
                    if self.out_n >= 2:
                        self.out_n = 0
                        self.move(out, now, tgt["pan"], tgt["tilt"])
                else:
                    self.out_n = 0
                    self.errs = (self.errs + [(dx, dy)])[-5:]
                    if len(self.errs) >= 3 and now - self.moved_t >= FINE_WAIT:
                        mx = sum(e[0] for e in self.errs) / len(self.errs)
                        my = sum(e[1] for e in self.errs) / len(self.errs)
                        if abs(mx) > FINE or abs(my) > FINE:
                            self.move(out, now, self.pan + mx, self.tilt + my, settle=0.4)
        elif self.mode == "idle" and now > self.until:
            self.until = now + self.rng.uniform(*IDLE_EVERY)
            side = self.rng.choice((-1, 1)) * self.rng.uniform(*IDLE_OFF)
            self.move(out, now, self.tpan + side, self.ttilt + self.rng.uniform(-10, 10))
        elif self.mode == "scan":
            if visible:
                self.attend(out, now, "found", max(visible, key=lambda p: p["px"]))
            elif now - self.seen_t > SCAN_AFTER and now > self.until:
                self.until = now + self.rng.uniform(*SCAN_EVERY)
                choices = [x for x in SCAN_PANS if abs(x - self.pan) > 20.0]
                self.move(out, now, self.rng.choice(choices), self.rng.uniform(5.0, 25.0), settle=1.0)
        return out

    def attend(self, out, now, why, p):
        self.target, self.out_n = p["id"], 0
        dur = GLANCE_S if why == "glance" else self.rng.uniform(*ATTEND_S)
        self.until = now + dur
        self.set_mode("attend", why, now)
        self.peek, self.peek_hidden, self.attend_t = (now, p["pan"], p["tilt"]), False, now
        if why == "approach" and p.get("base"):             # ④ 움찔 판단용: 다가오기 전 얼굴 크기 (1.2 s 안에 1.6배면 움찔)
            self.approach0 = (now, p["base"], p["id"])
        self.move(out, now, p["pan"], p["tilt"])
        if why == "arrive":
            self.nod(now, p["pan"], p["tilt"])

    def ready(self, name, now):
        """클립 쿨다운 확인 + 시작 표시"""
        if now - self.cool.get(name, -1e9) < COOL[name]:
            return False
        self.cool[name] = now
        return True

    def clips(self, out, now, people, tgt):
        """attend 중 행동 클립 ①②⑤ (④ 는 attend() 에서). 시작했으면 True"""
        back, self.peek_back = self.peek_back, None
        if back and now - back[0] < 0.5:                     # ① 까꿍: 같은 자리 10° 안에서 다시 나타남
            p = back[1]
            if True:
                if p["px"] >= 20.0 and self.ready("peekaboo", now):
                    self.target = p["id"]
                    pn, tl = p["pan"], p["tilt"]
                    self.play(now, [(0.0, ("move", pn, tl + 6.0, 0, 0, 0.3)),             # 놀람 (젖힘)
                                    (0.35, ("move", pn, tl - 7.0, 0, 0, 0.25)), (0.65, ("move", pn, tl, 0, 0, 0.25)),
                                    (0.95, ("move", pn, tl - 7.0, 0, 0, 0.25)), (1.25, ("move", pn, tl, 0, 0, 0.3)),
                                    (0.4, ("event", "peekaboo"))])
                    return True
        if not tgt or tgt["unseen"] > 0.1 or tgt["px"] < CLIP_MIN_PX:
            return False
        a0 = self.approach0
        if a0 and a0[2] == tgt["id"] and now - a0[0] <= 1.2 and tgt["px"] >= FLINCH_RATIO * a0[1]:
            self.approach0 = None                            # ④ 움찔: 아주 빠른 다가옴 → 뒤로 젖힘 → 올려다봄
            if self.ready("flinch", now):
                self.play(now, [(0.0, ("move", self.pan, self.tilt + 5.0, 0, 0, 0.25)),
                                (0.3, ("move", tgt["pan"], tgt["tilt"] + 2.0, 0, 0, 0.4)),
                                (0.0, ("event", "flinch"))])
                return True
        m = self.mem.get(tgt["id"])
        if not m:
            return False
        sm = [(t, y) for t, y in m["smooth"] if t >= m["away_t"]]
        held = now - m["away_t"]
        if (not m["contact"] and FOLLOW_HOLD <= held <= FOLLOW_HOLD + 0.5 and self.follow_done != m["away_t"] and sm
                and all(abs(y) > FOLLOW_YAW for _, y in sm) and len({y > 0 for _, y in sm}) == 1):
            self.follow_done = m["away_t"]                   # ② 시선 따라가기 (이번 고개 돌림에 한 번)
            if self.ready("follow_gaze", now):
                side = 1.0 if sm[-1][1] > 0 else -1.0
                self.play(now, [(0.0, ("move", tgt["pan"] + side * FOLLOW_OFF, tgt["tilt"], 0, 0, 0.3)),
                                (1.5, ("move", tgt["pan"], tgt["tilt"], 0, 0, 0.4)),
                                (0.1, ("event", "follow_gaze"))])
                return True
        if NOD_ON and m["contact"]:                          # ⑤ 따라 끄덕
            ps = [x for t, x in m["pitch"] if now - t <= 1.5]
            if len(ps) >= 5 and max(ps) - min(ps) >= 20.0 and abs(ps[-1] - ps[0]) < 6.0 and self.ready("nod", now):
                self.nod(now - 0.2, self.pan, self.tilt)     # nod 는 0.5 s 뒤 시작 → 약 0.3 s 뒤
                self.clip.append((now + 0.3, ("event", "nod")))
                self.clip.sort(key=lambda x: x[0])
                return True
        return False

    def nod_off(self, now, last=False):
        """꾸벅 한 번 (A·B 랜덤) 예약. last 면 깨지 않고 끝까지 숙여 잠들기 → 끝나는 데 걸리는 시간"""
        td, pn = self.base_tilt, self.pan
        sink, sink_s = self.rng.uniform(8.0, 12.0), self.rng.uniform(4.0, 7.0)   # 더 길고 느리게 = 이어지는 졸음
        drop = self.rng.uniform(6.0, 10.0)
        spd = max(5, int(sink / sink_s * STEPS_PER_DEG))
        steps, t = [], 0.0
        if self.rng.random() < 0.5:                          # A: 툭(작게) 먼저
            td -= 4.0
            steps.append((t, ("move", pn, td, 0, 0, 0.3)))
            t += 0.4
            hold = self.rng.uniform(0.3, 0.5)
        else:                                                # B: 바로 스르르륵, 멈칫이 조금 더 김
            hold = self.rng.uniform(0.5, 0.8)
        steps.append((t, ("move", pn, td - sink, spd, 2, sink_s)))              # 스르르륵
        t += sink_s + 0.1
        steps.append((t, ("move", pn, td - sink - drop, 0, 0, 0.3)))           # 툭
        t += 0.3 + hold
        if last:                                             # 마지막: 안 깨고 그대로 끝까지 숙임 → 잠
            steps.append((t, ("move", pn, TILT_SLEEP, *SLOW, 3.0)))
            t += 3.2
            steps.append((t, ("event", "asleep")))
        else:
            up = self.base_tilt + 3.0                        # 깸: 원래보다 살짝 위로 (화들짝 조금)
            steps.append((t, ("move", pn, up, 0, 0, 0.3)))
            t += 0.8
            steps.append((t, ("move", pn, self.base_tilt, *SLOW, 1.0)))
            t += 1.2
            self.base_tilt -= self.rng.uniform(2.0, 3.0)     # 꾸벅 사이 아주 느린 가라앉음
            steps.append((t, ("move", pn, self.base_tilt, 10, 1, 1.0)))         # SPD 10 ≈ 0.9°/s
        self.play(now, steps)
        return t

    def doze(self, out, now, why, nods=3):
        self.set_mode("doze", why, now)
        self.nods_left, self.base_tilt = nods, TILT_DOZE
        self.until = now + self.rng.uniform(1.5, 3.0)        # 졸린 자세로 간 뒤 첫 꾸벅까지
        self.move(out, now, self.pan, TILT_DOZE, *SLOW, settle=2.0)
        out.append(("led", LED_DOZE))

    def wake(self, out, now, why, p=None, strength="strong", direction=None):
        """깨기: strong = 번쩍 + 자극 쪽(direction, 팬 °)으로 / weak = 천천히 고개 들고 두리번"""
        self.clip = []
        pn = self.pan if direction is None else direction
        if strength == "strong":
            out.append(("led", LED_FLASH))
            self.play(now, [(0.5, ("led", LED_AWAKE))])
        if p is not None:
            self.attend(out, now, why, p)
            return
        self.set_mode("scan", "wake:" + why, now)
        if strength == "strong":
            self.until = now + 1.5
            self.move(out, now, pn, TILT_HOME + 5.0)
        else:
            self.until = now + 3.0
            self.move(out, now, pn, TILT_HOME, *SLOW, settle=2.5)
        self.seen_t = min(self.seen_t, now - SCAN_AFTER)    # 깨자마자 두리번
