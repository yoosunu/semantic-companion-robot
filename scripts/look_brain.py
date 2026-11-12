"""look_brain — 고개의 "마음" (ROS 없음, 순수 계산). look_at_person 노드가 씀, tests/look_brain_sim.py 로 시험.

  원칙 (11-12 Jasper): 사람이 있다고 계속 쳐다보면 감시 카메라. **쳐다볼 이유가 있을 때만** 쳐다본다.

  모드
    scan    아무도 없음 → 4~7 s 마다 좌우 한 곳 두리번
    attend  사람을 쳐다봄 (가운데 ±10°/±8° 면 가만히, 밖이면 한 번에 확). 10~30 s (매번 랜덤) 지나면 → idle
    idle    딴짓: 사람은 곁눈(화면 안)에 두고 25~45° 옆을 봄, 5~12 s 마다 바꿈. 트리거가 오면 → attend
    doze    꾸벅: 1 분 동안 아무 일(트리거) 없으면. 고개 천천히 내려가고 LED 어둡게, 가끔 꾸벅. 트리거 → 번쩍 깸
    sleep   잠: 꾸벅 3 분 더. 고개 끝까지 숙이고 LED 끔, 얼굴 처리도 멈춤 (보안·절약). LiDAR 로 가까운 움직임 → 깸

  다시 쳐다볼 이유 (트리거) — 이게 있어야 attend 로
    contact   사람이 나를 쳐다보기 시작 (|yaw| 35° 넘던 사람이 20° 안으로)
    glance    사람이 두리번 (1 s 안에 yaw 25° 넘게 바뀜) → 짧게 3 s 만 봄
    approach  다가옴 (1.5 s 안에 얼굴 1.4 배)
    arrive    새 사람 (10 s 동안 아무도 없다가) → 쳐다보고 끄덕 인사
  단순히 "있음" 은 트리거가 아님 → 사람이 컴퓨터만 보고 있으면 로봇은 딴짓하다 졸게 됨.

  출력 동작: ("move", 팬, 틸트, SPD, ACC)  SPD/ACC 0 = 서보 최고 속도 / ("led", 0~255) / ("perceive", bool)
"""
import collections
import random

ZONE_X, ZONE_Y = 10.0, 8.0           # attend: 이 안이면 가만히
SETTLE = 0.6                         # 빠른 고개 이동 뒤 기다림
ATTEND_S = (10.0, 30.0)              # 한 번 쳐다보면 이만큼 (랜덤)
GLANCE_S = 3.0
IDLE_EVERY = (5.0, 12.0)             # 딴짓 시선 바꾸는 간격
IDLE_OFF = (25.0, 45.0)              # 딴짓: 사람에서 이만큼 옆 (화각 ±60° 안 → 사람이 화면에 남아 계속 지켜봄)
SCAN_AFTER = 5.0
SCAN_EVERY = (4.0, 7.0)
SCAN_PANS = (-70.0, -35.0, 0.0, 35.0, 70.0)
LOST_S = 2.0
DOZE_AFTER = 60.0                    # 11-12 Jasper: 꾸벅 1 분
SLEEP_AFTER = 180.0                  # 꾸벅 뒤 3 분 (짧으면 5 분)
CONTACT_IN, CONTACT_OUT = 20.0, 35.0
GLANCE_YAW = 25.0
APPROACH_RATIO, APPROACH_WIN = 1.4, 1.5
ARRIVE_GAP = 10.0
LED_AWAKE, LED_DOZE, LED_FLASH = 40, 8, 255
SLOW = (150, 5)                      # 꾸벅용 느린 SPD, ACC (ST3215: SPD 걸음/s, 4096 걸음 = 한 바퀴 → 150 ≈ 13°/s)
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
        self.event_t = 0.0                   # 마지막 트리거 (꾸벅 타이머)
        self.seen_t = -1e9                   # 마지막으로 누군가 보인 시각
        self.doze_t = 0.0
        self.out_n = 0                       # attend: 가운데 밖으로 연속 몇 번
        self.mem = {}                        # 트랙별 {"yaw": deque[(t,yaw)], "px": deque[(t,px)], "contact": bool}
        self.started = False

    # ---------- 작은 도구 ----------
    def move(self, out, now, pan, tilt, spd=0, acc=0, settle=SETTLE):
        pan = (pan + 180.0) % 360.0 - 180.0
        self.pan, self.tilt = pan, max(-30.0, min(90.0, tilt))
        out.append(("move", self.pan, self.tilt, spd, acc))
        self.busy_until = now + settle

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
    def triggers(self, now, people):
        """사람마다 이번에 생긴 트리거 → [(이름, 트랙)]"""
        found = []
        for p in people:
            if p["unseen"] > 0:
                continue
            m = self.mem.setdefault(p["id"], {"yaw": collections.deque(maxlen=30), "px": collections.deque(maxlen=30),
                                              "contact": abs(p["yaw"]) <= CONTACT_IN, "new": True})
            m["yaw"].append((now, p["yaw"]))
            m["px"].append((now, p["px"]))
            if m.pop("new", False):
                continue                                     # 첫 관측은 기준만
            if not m["contact"] and abs(p["yaw"]) <= CONTACT_IN:
                m["contact"] = True
                found.append(("contact", p))
            elif m["contact"] and abs(p["yaw"]) >= CONTACT_OUT:
                m["contact"] = False
            old = [y for t, y in m["yaw"] if now - t <= 1.0]
            if old and max(old) - min(old) >= GLANCE_YAW:
                found.append(("glance", p))
                m["yaw"].clear()
            olds = [x for t, x in m["px"] if now - t <= APPROACH_WIN]
            if olds and p["px"] >= APPROACH_RATIO * min(olds):
                found.append(("approach", p))
                m["px"].clear()
        ids = {p["id"] for p in people}
        for k in [k for k in self.mem if k not in ids]:
            del self.mem[k]
        return found

    # ---------- 한 번 (0.1 s 마다) ----------
    def tick(self, now, people, motion=False):
        """people: [{"id","pan","tilt","px","yaw","pitch","unseen"}] (/people 최신), motion: LiDAR 가까운 움직임"""
        out = []
        if not self.started:
            self.started, self.event_t = True, now
            out.append(("led", LED_AWAKE))
            self.move(out, now, 0.0, TILT_HOME)
        while self.clip and now >= self.clip[0][0]:
            a = self.clip.pop(0)[1]
            if a[0] == "move":
                self.move(out, now, a[1], a[2], a[3], a[4], a[5])
            else:
                out.append(a)

        if self.mode == "sleep":                            # 잠: LiDAR 만 봄
            if motion:
                self.wake(out, now, "lidar")
            return out

        visible = [p for p in people if p["unseen"] <= 0.5]
        if visible:
            if now - self.seen_t > ARRIVE_GAP:
                trig = [("arrive", max(visible, key=lambda p: p["px"]))]
            else:
                trig = []
            self.seen_t = now
            trig += self.triggers(now, visible)
        else:
            trig = []

        if self.mode == "doze":
            if trig:
                self.wake(out, now, trig[0][0], trig[0][1])
            elif now - self.doze_t > SLEEP_AFTER:
                self.set_mode("sleep", "no event", now)
                self.clip = []
                self.move(out, now, self.pan, TILT_SLEEP, *SLOW, settle=4.0)
                out += [("led", 0), ("perceive", False)]
            elif not self.clip and now > self.until:        # 가끔 꾸벅
                self.until = now + self.rng.uniform(8.0, 15.0)
                self.play(now, [(0.0, ("move", self.pan, TILT_DOZE - 12.0, *SLOW, 1.5)),
                                (1.6, ("move", self.pan, TILT_DOZE, 0, 0, 0.4))])
            return out

        if trig:
            self.event_t = now
            name, p = trig[0]
            if self.clip and name != "arrive":
                pass                                         # 인사 중이면 끝까지
            else:
                self.attend(out, now, name, p)
        elif now - self.event_t > DOZE_AFTER and not self.clip and now >= self.busy_until:
            self.set_mode("doze", "no event", now)
            self.doze_t, self.until = now, now + self.rng.uniform(8.0, 15.0)
            self.move(out, now, self.pan, TILT_DOZE, *SLOW, settle=2.0)
            out.append(("led", LED_DOZE))
            return out

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
        if self.mode == "attend":
            if now > self.until:
                self.set_mode("idle", "bored", now)
                self.until = 0.0
            elif tgt and tgt["unseen"] <= 0.5:
                dx, dy = tgt["pan"] - self.pan, tgt["tilt"] - self.tilt
                if abs(dx) > ZONE_X or abs(dy) > ZONE_Y:
                    self.out_n += 1
                    if self.out_n >= 2:
                        self.out_n = 0
                        self.move(out, now, tgt["pan"], tgt["tilt"])
                else:
                    self.out_n = 0
        elif self.mode == "idle" and now > self.until:
            self.until = now + self.rng.uniform(*IDLE_EVERY)
            base = tgt["pan"] if tgt else self.pan
            side = self.rng.choice((-1, 1)) * self.rng.uniform(*IDLE_OFF)
            self.move(out, now, base + side, (tgt["tilt"] if tgt else TILT_HOME) + self.rng.uniform(-10, 10))
        elif self.mode == "scan":
            if visible:
                self.attend(out, now, "found", max(visible, key=lambda p: p["px"]))
            elif now - self.seen_t > SCAN_AFTER and now > self.until:
                self.until = now + self.rng.uniform(*SCAN_EVERY)
                choices = [x for x in SCAN_PANS if abs(x - self.pan) > 20.0]
                self.move(out, now, self.rng.choice(choices), self.rng.uniform(5.0, 25.0), settle=1.0)
        return out

    def attend(self, out, now, why, p):
        self.target, self.out_n, self.event_t = p["id"], 0, now
        dur = GLANCE_S if why == "glance" else self.rng.uniform(*ATTEND_S)
        self.until = now + dur
        self.set_mode("attend", why, now)
        self.move(out, now, p["pan"], p["tilt"])
        if why == "arrive":
            self.nod(now, p["pan"], p["tilt"])

    def wake(self, out, now, why, p=None):
        """번쩍: 빠르게 고개 들기 + LED 반짝"""
        self.event_t, self.clip = now, []
        out += [("perceive", True), ("led", LED_FLASH)]
        self.play(now, [(0.5, ("led", LED_AWAKE))])
        if p is not None:
            self.attend(out, now, why, p)
        else:
            self.set_mode("scan", "wake:" + why, now)
            self.until = now + 1.5
            self.move(out, now, self.pan, TILT_HOME + 5.0)
            self.seen_t = min(self.seen_t, now - SCAN_AFTER)    # 깨자마자 두리번


class Motion:
    """LiDAR 가까운 움직임 감지 (잠잘 때 깨우기). 로봇이 서 있다는 가정.
       기준 스캔과 비교해 near_m 안에서 0.25 m 넘게 바뀐 빔이 n 개 이상인 스캔이 2 번 연속이면 움직임."""

    def __init__(self, near_m=1.5, diff_m=0.25, n=5):
        self.near, self.diff, self.n = near_m, diff_m, n
        self.base, self.hits, self.base_t = None, 0, 0.0

    def step(self, now, ranges):
        r = [x if (x == x and 0.05 < x < 12.0) else 12.0 for x in ranges]   # nan·inf·0 → 멀다
        if self.base is None or len(self.base) != len(r) or now - self.base_t > 30.0 and self.hits == 0:
            self.base, self.base_t = r, now                  # 30 s 조용하면 기준 새로 (천천히 바뀐 것 흡수)
            return False
        changed = sum(1 for a, b in zip(r, self.base) if min(a, b) < self.near and abs(a - b) > self.diff)
        self.hits = self.hits + 1 if changed >= self.n else 0
        if self.hits >= 2:
            self.base, self.base_t, self.hits = r, now, 0
            return True
        return False
