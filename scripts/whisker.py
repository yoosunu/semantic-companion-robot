#!/usr/bin/env python3
"""Whisker (수염) — 기어다니는 로봇의 촉각. LiDAR(19.5 cm) 아래 것(의자 다리, 건조대 다리, 레일, 턱)을 닿아서 안다.

  느끼기   /imu + /motor_out(브리지가 방금 모터를 바꿨나 = efference copy) → 부딪힘 / 끄덕임 / 건드림
           + 예상보다 느림(odom): 명령만큼 안 움직임 → 충격 없는 막힘 (손·쿠션·동물)
  단단/부드러움  충격·끄덕임 = 단단 → 영구 표시 / 느려짐·한쪽 들림(밟음) = 부드러움 → 바로 물러남, 표시 1분
  느려짐 (출발 1.5 s 뒤부터): 한 번 = 부드러운 것에 막힘 → 바로 5 cm 물러남 + 1분 표시 /
           5 s·20 cm 안에 2번 이상 = stuck (헛바퀴·카펫 가장자리) → 바로 10 cm 빼기 + 차체 폭 영구 표시
  힘 제한  브리지 v8.1: PWM ≤ 40 직진 / ≤ 55 회전 — 감지가 실패해도 짓누르지 않게 (협동 로봇의 힘 제한 원칙)
  방향     직진 중 = 가던 쪽 (확실 → 표시). 회전·정지 중 = IMU 참고 (11-06: 박을 때 출렁임 + IMU ~10 Hz → 못 믿음, 표시 안 함)
  반응     멈춤 → 왔던 쪽으로 10 cm 빼기 (회전 중이었으면 15° 되돌리기) → 성공하면 Nav2 에 돌려줌
  갇힘     빼다가 또 닿음 → 10초 가만히 → 다시 빼기 (1: 같은 방법, 2: 15° 돌고 빼기, 3: 반대로 15° 돌고 빼기)
           → 3번 다 실패하면 구조 요청 (ntfy) + 정지. 풀기: ros2 topic pub --once /whisker/resume std_msgs/msg/Empty
  기억     닿은 점을 장애물 지도에 (/whisker/cloud → costmap whisker_layer, LiDAR 가 못 지움). 좁게: 닿은 점 ~10 cm.
           지우기: ros2 topic pub --once /whisker/clear std_msgs/msg/Empty
  기록     ~/scr_logs/whisker/<날짜>.jsonl, 로그 줄 "SENSE {...}" (run_report 가 셈)

브리지는 얇게 (v8): /imu, /motor_out 을 내고, /whisker/cmd_vel 이 오는 동안은 Nav2 를 무시한다. 판단은 전부 여기.
문턱값·거리는 ROS 파라미터 → 재빌드 없이 `ros2 param set /whisker bump_g 0.05`. 측정 근거: docs/calibration/imu-touch-2026-11-06.md
IMU 로 충격 방향 알기는 연구 과제로 남김 (펌웨어 IMU 갱신 속도 확인 → 다시 도전). ax_forward / ay_left 는 그때 쓸 것.
"""
import collections
import json
import math
import struct
import threading
import time
import urllib.request
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu, PointCloud2, PointField
from std_msgs.msg import Empty, Float32MultiArray, String

G = 9.81
HALF_L, HALF_W = 0.13, 0.12   # m, 차체 반길이·반폭 (footprint)
MARK_GAP = 0.03               # m, 몸 바깥으로
MARK_HALF = 0.05              # m, 표시 반폭 → 점 3개, 폭 ~10 cm
MARK_Z = 0.10                 # m, 장애물 높이 (costmap 높이 필터 안)
SELF_MASK = 0.4               # s, 우리가 모터를 바꾼 뒤 IMU 무시 (출발 흔들림 0.17 g 은 우리 것)
MOVING_V, MOVING_W = 0.025, 0.10   # m/s, rad/s — 직전 0.5 s 동안 "실제로" 이만큼 움직였으면 = 굴러가는 중 (거리계·자이로)
START_BLOCK_S = 1.0           # s, 출발 명령 후 이 시간 동안 1 cm 눈금이 하나도 안 뜨면 = 앞이 막힘 (바퀴 떼기 0.3~0.8 s)
BACK_SPEED, TURN_SPEED = 0.05, 0.15   # m/s, rad/s — 바닥선(0.05 / 0.13) 바로 위, 연속으로 잔잔하게.
                                      #   11-06: 0.25 rad/s 는 출발 밀기 + 자이로 보정으로 과격. TAP 은 0.5 s 마다
                                      #   모터가 바뀌어 감지가 0.4 s 씩 가려짐 → 빼는 중 다시 박아도 놓침 → 안 씀
STEP_TIMEOUT = 8.0            # s, 빼기 한 번 최대
SLOW_WINDOW = 0.5             # s, "예상보다 느림" 판단 창 (0.05 m/s 면 1 cm 눈금 2.5개가 떠야 함)
SLOW_SETTLE = 1.5             # s, 명령이 이만큼 이어진 뒤부터 판단. 11-06: 0.8 → 1.5 (출발 헛것이 1.2~1.7 s 에 뜸).
                              #   그 사이 안전은 힘 제한(PWM 40/55)이 지킴
SLOW_RATIO = 0.3              # 예상 이동의 30% 도 못 가면 = 막힘
SOFT_TTL = 60.0               # s, 부드러운 것(손·발·동물일 수도) 표시는 1분 뒤 사라짐
RELEASE_FRAC = 0.6            # 빼기 거리의 앞 60% 동안 사건 무시 = 올라탔던 것에서 내려오는 흔들림
STUCK_N, STUCK_S, STUCK_R = 2, 5.0, 0.20   # "느림"이 5 s 안에, 20 cm 안에서 2번 이상 = 진짜 막힘·헛바퀴 (11-06 카펫:
                                           #   1초마다 연속 / 출발 헛것은 한 번만) → 못 가는 구역
STUCK_HALF = 0.12             # m, 막힘 표시 반폭 → 차체 폭 ~25 cm (카펫 가장자리·턱 = 선)


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def side_name(ang):
    a = math.degrees(math.atan2(math.sin(ang), math.cos(ang)))
    for lo, hi, n in ((-45, 45, "front"), (45, 135, "left"), (-135, -45, "right")):
        if lo <= a < hi:
            return n
    return "rear"


class Whisker(Node):
    def __init__(self):
        super().__init__("whisker")
        p = self.declare_parameter
        self.bump_g = p("bump_g", 0.06).value          # 주행 중 수평 가속 튐 (상자 0.09, 주행 잡음 0.02)
        self.pitch_dps = p("pitch_dps", 5.5).value     # 주행 중 앞뒤 끄덕임 (카펫 7)
        self.roll_dps = p("roll_dps", 5.0).value       # 주행 중 좌우 기우뚱 = 한쪽 궤도가 뭔가를 밟음 (주행 중 gx 최대 ~2.7)
        self.slow_react = p("slow_react", True).value  # "예상보다 느림" 에 반응. 11-06 밤: 관찰만(False)으로 뒀더니 카펫 가장자리에서
                                                       #   17번 알고도 안 움직임 → Jasper 의도("부드러운 건 바로 물러남, 헛돌면 못 가는 곳")대로 켬
        self.touch_g = p("touch_g", 0.025).value       # 정지 중 건드림 (톡 0.04)
        self.rock_dps = p("rock_dps", 4.0).value       # 정지 중 기우뚱 (톡 6)
        self.back_dist = p("back_dist", 0.10).value    # m, 빼는 거리 (단단한 것)
        self.soft_dist = p("soft_dist", 0.05).value    # m, 부드러운 것: 즉시 이만큼 물러남
        self.back_turn = math.radians(p("back_turn_deg", 15.0).value)
        self.wait_s = p("trapped_wait", 10.0).value
        self.ax_forward = p("ax_forward", 1.0).value   # 보드 +x 가 로봇 앞이면 +1 — 11-06 정면 박기 2번이 반대 부호라 미확정
        self.ay_left = p("ay_left", 1.0).value         #   (IMU 방향은 회전·정지 중 참고용으로만 씀)
        self.react = p("react", True).value            # False = 느끼고 기록만 (관찰 모드)

        # 감각 상태
        self.base = None                     # [ax, ay, gx, gy] 평소 값 (천천히 따라감)
        self.motor = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.prev_motor = None
        self.motor_hist = collections.deque()   # (시각, pwm_l, pwm_r, cmd_x, cmd_z) 최근 0.4 s
        self.start = None                       # {"t", "pos"} 서 있다 출발한 순간
        # 지나온 길 (11-07): 빼기는 "방금 지나온 곳"으로만, 지나온 만큼만. 출발 직후엔 온 길이 없음 → 안 뺌
        self.trail = {"sign": 0, "len": 0.0, "last": None, "idle": None}
        self.last_self = 0.0                 # 우리가 마지막으로 모터를 바꾼 시각
        self.last_event = 0.0
        # 위치
        self.odom = None                     # (x, y, yaw) odom
        self.odom_hist = collections.deque(maxlen=20 * 180)
        self.map_fix = None
        # 반응 상태 기계
        self.state = "drive"                 # drive / stop / back / trapped / help
        self.state_t = 0.0
        self.step = None                     # 지금 빼기 계획 {"kind": "lin"/"rot", "sign", "goal", "start", "pre_rot"}
        self.contact_ang = 0.0               # "처음" 닿은 방향 (몸 기준 rad, 0 = 앞) — 갇혀도 안 바꿈 (11-06 핑퐁 수정)
        self.contact_feel = "hard"
        self.contact_kind = "bump"
        self.plan_text = ""
        self.last_cmd = (0.0, 0.0)           # 닿기 직전 우리가 따르던 명령 (x, z)
        self.trapped = 0
        self.left_hits, self.right_hits = 0, 0
        self.cmd_since = time.time()       # 지금 명령이 시작된 시각 (느려짐 판단용)
        self.slow_hist = collections.deque(maxlen=10)   # (시각, x, y) — 느림이 연달아 뜨는지
        self.cmd_now = (0.0, 0.0)
        # 기억
        self.points = []                   # (x, y, 만료 시각 또는 None=영구)
        self.contacts = 0

        self.create_subscription(Imu, "imu", self.on_imu, 50)
        self.create_subscription(Float32MultiArray, "motor_out", self.on_motor, 50)
        self.create_subscription(Odometry, "odom", self.on_odom, 20)
        for t in ("pose", "amcl_pose"):
            self.create_subscription(PoseWithCovarianceStamped, t, self.on_map_pose, 10)
        self.create_subscription(Empty, "whisker/clear", self.on_clear, 1)
        self.create_subscription(Empty, "whisker/resume", self.on_resume, 1)
        self.cmd_pub = self.create_publisher(TwistStamped, "whisker/cmd_vel", 10)
        self.cloud_pub = self.create_publisher(PointCloud2, "whisker/cloud", 1)
        self.soft_pub = self.create_publisher(PointCloud2, "whisker/soft_cloud", 1)   # RViz 색 구분용 (임시 표시만)
        self.sense_pub = self.create_publisher(String, "sense", 10)
        self.state_pub = self.create_publisher(String, "whisker/state", 10)   # 상태·계획 (whisker_lab.py 가 보여줌)
        self.create_timer(0.05, self.tick)
        self.create_timer(1.0, self.publish_cloud)
        d = Path.home() / "scr_logs" / "whisker"
        d.mkdir(parents=True, exist_ok=True)
        self.log_path = d / f"{time.strftime('%Y-%m-%d')}.jsonl"
        self.get_logger().info(f"whisker ready (react={self.react}) → log {self.log_path}")

    # ------------------------------------------------------------------ 입력
    def on_motor(self, m):
        d = list(m.data) + [0.0] * 5
        self.motor = d[:5]
        # efference copy: "방금 우리가 모터를 바꿨나". 11-07 수정: 한 샘플 차이만 보면 Nav2 의 부드러운 가속(매 틱 조금씩)과
        #   바퀴 떼기 밀기(매 틱 +3 PWM)를 못 잡아서, 출발 흔들림(0.17 g)을 부딪힘으로 오해 (imu_lab 은 계단 명령이라 안 드러남).
        #   → 0.4 s 전과 비교 + 멈춰 있다 출발하면 1 s 동안 가림
        now = time.time()
        self.motor_hist.append((now, d[0], d[1], d[2], d[3]))
        while self.motor_hist and now - self.motor_hist[0][0] > SELF_MASK:
            self.motor_hist.popleft()
        o = self.motor_hist[0]
        if (abs(d[0] - o[1]) > 8 or abs(d[1] - o[2]) > 8 or abs(d[2] - o[3]) > 0.02 or abs(d[3] - o[4]) > 0.05):
            self.last_self = max(self.last_self, now)
        # 출발 기록: 모터가 꺼져 있다 켜진 순간의 시각·자리 → "출발했는데 안 나가짐" 판단용
        on = abs(d[0]) > 1 or abs(d[1]) > 1
        was_on = self.prev_motor is not None and (abs(self.prev_motor[0]) > 1 or abs(self.prev_motor[1]) > 1)
        if on and not was_on and self.odom:
            self.start = {"t": now, "pos": self.odom}
        elif not on:
            self.start = None
        if abs(d[2] - self.cmd_now[0]) > 0.02 or abs(d[3] - self.cmd_now[1]) > 0.05:
            self.cmd_now, self.cmd_since = (d[2], d[3]), time.time()
        self.prev_motor = d[:5]

    def on_odom(self, m):
        st, pp = m.header.stamp, m.pose.pose
        self.odom = (pp.position.x, pp.position.y, yaw_of(pp.orientation))
        self.odom_hist.append((st.sec + st.nanosec * 1e-9, *self.odom))
        self.update_trail()
        self.check_start_blocked()
        self.check_slow()

    def update_trail(self):
        """Nav2 가 운전하는 동안(drive) 같은 방향(앞/뒤)으로 이어서 간 거리. 방향이 바뀌거나 1 s 넘게 서 있으면 처음부터"""
        if self.state != "drive" or not self.odom:
            return
        tr, cx, now = self.trail, self.motor[2], time.time()
        sign = 1 if cx > 0.01 else (-1 if cx < -0.01 else 0)
        pos = self.odom[:2]
        if sign == 0:
            tr["idle"] = tr["idle"] or now
            if now - tr["idle"] > 1.0:
                tr.update(sign=0, len=0.0, last=pos)
            return
        tr["idle"] = None
        if sign != tr["sign"] or tr["last"] is None:
            tr.update(sign=sign, len=0.0, last=pos)
        else:
            tr["len"] += math.hypot(pos[0] - tr["last"][0], pos[1] - tr["last"][1])
            tr["last"] = pos

    def moving_now(self):
        """직전 0.5 s 동안 실제로 움직였나 (바퀴 거리계 / 자이로) — 명령이 아니라 측정"""
        if len(self.odom_hist) < 3:
            return False
        t1, x1, y1, a1 = self.odom_hist[-1]
        old = next((r for r in reversed(self.odom_hist) if t1 - r[0] >= 0.5), None)
        if not old:
            return False
        dt = t1 - old[0]
        v = math.hypot(x1 - old[1], y1 - old[2]) / dt
        w = abs(math.atan2(math.sin(a1 - old[3]), math.cos(a1 - old[3]))) / dt
        return v >= MOVING_V or w >= MOVING_W

    def check_start_blocked(self):
        """출발했는데 안 나가짐: 명령 후 1 s 동안 1 cm 눈금도, 3° 회전도 없음 = 앞(또는 뒤)이 막힘. (집에서 나오는데 앞에 뭐가 있을 때)"""
        st, now = self.start, time.time()
        if not st or not self.odom or self.state not in ("drive", "back") or now - st["t"] < START_BLOCK_S:
            return
        self.start = None                                   # 출발 한 번에 한 번만
        x0, y0, a0 = st["pos"]
        moved = math.hypot(self.odom[0] - x0, self.odom[1] - y0)
        turned = abs(math.atan2(math.sin(self.odom[2] - a0), math.cos(self.odom[2] - a0)))
        if moved >= 0.01 or turned >= 0.05:
            return
        cx, cz = self.motor[2], self.motor[3]
        if abs(cx) < 0.02 and abs(cz) < 0.1:
            return
        straight = abs(cx) >= 0.02
        ang = (0.0 if cx > 0 else math.pi) if straight else 0.0
        self.last_event = now
        self.slow_hist.append((now, self.odom[0], self.odom[1]))     # 같은 자리에서 또 막히면 → stuck (못 가는 곳, 영구)
        near = [h for h in self.slow_hist if now - h[0] <= STUCK_S * 2
                and math.hypot(h[1] - self.odom[0], h[2] - self.odom[1]) <= STUCK_R]
        if len(near) >= STUCK_N and straight:
            self.slow_hist.clear()
            self.on_contact({"ev": "stuck", "n": len(near), "dir": side_name(ang), "src": "motion", "cmd_x": round(cx, 3),
                             "cmd_z": round(cz, 2), "state": self.state, "why": "출발 막힘 반복"}, ang)
            return
        ev = {"ev": "blocked", "moved": round(moved, 3), "kind": "straight" if straight else "turn",
              "dir": side_name(ang) if straight else "turn", "src": "motion" if straight else "turn",
              "cmd_x": round(cx, 3), "cmd_z": round(cz, 2), "state": self.state}
        self.on_contact(ev, ang)

    def check_slow(self):
        """예상보다 느림 = 막힘. 충격이 없어도(손·쿠션·동물) 잡는다. 11-06: 빼다가 손을 밀어버림 → 추가.
        직진: 바퀴 1 cm 눈금이 예상만큼 안 뜸 / 회전: 자이로 각도가 예상만큼 안 돎 (자이로는 헛돌아도 정직)"""
        now = time.time()
        cx, cz = self.cmd_now
        # 직진 = 회전 성분이 거의 없을 때만. 11-06 23:57: 곡선(x 0.068, z -0.23)은 안쪽 바퀴가 거의 안 굴러 거리계 평균이
        #   명령의 절반 아래 → "막힘" 헛것. 곡선 중엔 판단 안 함. 제자리 회전은 자이로로 (헛돌아도 정직)
        straight = abs(cx) >= 0.04 and abs(cz) < 0.08
        turning = abs(cx) < 0.01 and abs(cz) >= 0.12   # 바닥선 아래(톡톡 주행)는 판단 안 함
        if not (straight or turning) or now - self.cmd_since < SLOW_SETTLE or now - self.last_event < 1.0:
            return
        if self.state not in ("drive", "back") or len(self.odom_hist) < 5:
            return
        t1, x1, y1, a1 = self.odom_hist[-1]
        old = next((r for r in reversed(self.odom_hist) if t1 - r[0] >= SLOW_WINDOW), None)
        if not old:
            return
        dt = t1 - old[0]
        if straight:
            got, want = math.hypot(x1 - old[1], y1 - old[2]), abs(cx) * dt
        else:
            got, want = abs(math.atan2(math.sin(a1 - old[3]), math.cos(a1 - old[3]))), abs(cz) * dt
        if got < SLOW_RATIO * want:
            self.last_event = now
            # 연달아 뜨면 진짜: 5 s 안·20 cm 안에서 2번 이상 → stuck (헛바퀴·카펫 가장자리) = 반응 + 넓은 영구 표시
            if self.odom:
                self.slow_hist.append((now, self.odom[0], self.odom[1]))
            near = [h for h in self.slow_hist if now - h[0] <= STUCK_S and self.odom
                    and math.hypot(h[1] - self.odom[0], h[2] - self.odom[1]) <= STUCK_R]
            if len(near) >= STUCK_N and straight:
                self.slow_hist.clear()
                ang = 0.0 if cx > 0 else math.pi
                ev = {"ev": "stuck", "got": round(got, 3), "want": round(want, 3), "n": len(near), "dir": side_name(ang),
                      "src": "motion", "cmd_x": round(cx, 3), "cmd_z": round(cz, 2), "state": self.state}
                self.on_contact(ev, ang)
                return
            if not self.slow_react:    # 한 번만 = 관찰만: 화면·로그에 숫자만 (출발 헛것일 수 있음)
                self.record({"ev": "slow", "observe": True, "got": round(got, 3), "want": round(want, 3),
                             "kind": "straight" if straight else "turn", "dir": "-", "feel": "soft", "cmd_x": round(cx, 3),
                             "cmd_z": round(cz, 2), "state": self.state, "marked": False})
                return
            ang = (0.0 if cx > 0 else math.pi) if straight else 0.0
            ev = {"ev": "slow", "got": round(got, 3), "want": round(want, 3), "kind": "straight" if straight else "turn",
                  "dir": side_name(ang) if straight else "turn", "src": "motion" if straight else "turn",
                  "cmd_x": round(cx, 3), "cmd_z": round(cz, 2), "state": self.state}
            self.on_contact(ev, ang)

    def on_map_pose(self, m):
        st, pp = m.header.stamp, m.pose.pose
        self.map_fix = (st.sec + st.nanosec * 1e-9, pp.position.x, pp.position.y, yaw_of(pp.orientation))

    def on_imu(self, m):
        now = time.time()
        a, w = m.linear_acceleration, m.angular_velocity
        s = [a.x / G, a.y / G, math.degrees(w.x), math.degrees(w.y)]
        if self.base is None:
            self.base = s[:]
            return
        d = [s[i] - self.base[i] for i in range(4)]
        horiz = math.hypot(d[0], d[1])
        motors_on = abs(self.motor[0]) > 1 or abs(self.motor[1]) > 1
        settled = now - self.last_self > SELF_MASK
        if settled and horiz < self.touch_g and abs(d[2]) < self.rock_dps and abs(d[3]) < self.rock_dps:
            self.base = [self.base[i] + 0.02 * d[i] for i in range(4)]   # 평소 값은 조용할 때만 천천히
        if not settled or now - self.last_event < 1.0:
            return
        ev = None
        # 11-07 (Jasper): 부딪힘은 "움직이다 박는 것" — 직전에 실제로 굴러가고 있었을 때만. 서 있다 출발할 때의 흔들림(0.17 g)은
        #   속도가 0 에서 막 올라가는 중이라 여기 안 걸림 (시간으로 가리는 게 아니라 물리 조건으로 거름)
        rolling = motors_on and self.moving_now()
        if rolling and horiz > self.bump_g:
            ev = {"ev": "bump", "g": round(horiz, 3)}
        elif rolling and abs(d[3]) > self.pitch_dps:
            ev = {"ev": "pitch", "dps": round(d[3], 1)}
        elif rolling and abs(d[2]) > self.roll_dps:   # 11-06: 바퀴 밑 손가락 = 한쪽만 들림 → 끄덕임보다 기우뚱
            ev = {"ev": "roll", "dps": round(d[2], 1)}
        elif not motors_on and (horiz > self.touch_g or abs(d[2]) > self.rock_dps or abs(d[3]) > self.rock_dps):
            ev = {"ev": "touch", "g": round(horiz, 3), "gx": round(d[2], 1), "gy": round(d[3], 1)}
        if not ev:
            return
        self.last_event = now
        # 충격 방향 (몸 기준).
        #   11-06 실측: 같은 정면 박기가 ax +0.09 / −0.07 로 반대 — 박는 순간 차체가 출렁이고, IMU 값은 ~10 Hz 로만 갱신
        #   (피드백 20 Hz 에 같은 값 두 번) → 한 샘플·0.15 s 합 모두 방향을 못 믿음.
        #   그래서: 직진 중이면 "가던 쪽" (앞으로 가다 뒤를 박을 순 없음) = 확실 → 빼기 + 표시.
        #           회전 중·정지 중이면 IMU 방향은 참고만 (src=imu) → 표시 안 함, 회전은 되돌리기만.
        if abs(self.motor[2]) >= 0.01:
            ang, src = (0.0 if self.motor[2] > 0 else math.pi), "motion"
        else:
            bx, by = self.ax_forward * d[0], self.ay_left * d[1]
            ang, src = (math.atan2(-by, -bx) if horiz > 1e-4 else 0.0), "imu"
        ev.update(dir=side_name(ang), src=src, ang=round(math.degrees(ang)), ax=round(d[0], 3), ay=round(d[1], 3),
                  cmd_x=round(self.motor[2], 3), cmd_z=round(self.motor[3], 2), state=self.state)
        self.on_contact(ev, ang)

    # ------------------------------------------------------------------ 반응
    def on_contact(self, ev, ang):   # (이름 주의: Node 가 이미 .handle 을 씀 → 11-06 첫 실행 TypeError)
        if self.state == "back" and self.backed_frac() < RELEASE_FRAC:
            # 빼기 초반의 흔들림 = 올라탔던 것(손가락·턱)에서 내려오는 것 (11-06: 손가락을 밟고 빼다가 "bump rear" → 갇힘 오해)
            self.get_logger().info(f"WHISKER 빼는 초반 흔들림 무시 (내려오는 중, {self.backed_frac() * 100:.0f}%)")
            return
        kind = ev["ev"]
        # 단단함 vs 부드러움: 충격(bump)·끄덕임(pitch) = 단단 → 영구 표시. 충격 없이 느려짐(slow) = 부드러움
        #   (쿠션·커튼·손·발·동물일 수도) → 밀지 않고 물러나고, 표시는 1분 뒤 사라짐 (살아 있는 건 움직이니까)
        #   roll(한쪽 궤도만 들림) = 뭔가를 밟음 (손가락·물건일 수도) → 부드러움 쪽으로 (표시 1분). pitch(양쪽) = 턱·레일 → 단단
        ev["feel"] = ("soft" if kind in ("slow", "roll", "blocked") else
                      "hard" if kind in ("bump", "pitch", "stuck") else "touch")   # stuck(헛바퀴 반복) = 못 가는 구역
        ttl = SOFT_TTL if ev["feel"] == "soft" else None
        marked = (kind in ("bump", "pitch", "roll", "slow", "stuck", "blocked") and ev.get("src") == "motion"
                  and self.mark(ang, ttl, STUCK_HALF if kind == "stuck" else MARK_HALF))   # 확실한 것만
        ev["marked"] = marked
        self.record(ev)
        if kind == "touch" or not self.react:
            return
        if ang > 0.3:
            self.left_hits += 1
        elif ang < -0.3:
            self.right_hits += 1
        if self.state == "drive":
            self.contact_ang, self.contact_feel, self.contact_kind = ang, ev["feel"], kind
            self.last_cmd = (self.motor[2], self.motor[3])
            self.trapped = 0
            # 바로 물러남: 부드러움(손·발일 수도) / 밟음·들림(pitch·roll — 올라탄 채 서 있지 않게). 부딪힘만 0.5 s 멈춘 뒤 빼기
            self.go("stop" if kind == "bump" else "back")   # stuck 도 바로 빼기 (헛돌며 버티지 않게)
        elif self.state == "back" and self.step and self.step["kind"] == "lin" and not self.step.get("rebound") \
                and self.backed() >= 0.02:
            # 빼다가 반대쪽에 닿음 (11-07 Jasper: "뒤에 박았으면 바로 앞으로 빼고 다음으로") → 기다리지 않고 바로 반대로,
            #   방금 뺀 만큼만 (최대 5 cm) — 그 길은 방금 지나왔으니 비어 있음
            back, sign = min(0.05, self.backed()), -self.step["sign"]
            self.step = {"kind": "lin", "sign": sign, "goal": back, "pre": 0.0, "start": self.odom, "phase": "main",
                         "rebound": True}
            self.plan_text = f"반대쪽에도 닿음 → 바로 {'앞으' if sign > 0 else '뒤'}로 {back * 100:.0f} cm (방금 지나온 길)"
            self.state_t = time.time()
            self.get_logger().warn(f"WHISKER back: 빼기 — {self.plan_text}")
            self.state_pub.publish(String(data=json.dumps({"state": "back", "msg": "빼기 — " + self.plan_text,
                                                           "try": self.trapped}, ensure_ascii=False)))
        elif self.state == "back":          # 빼고, 되돌아 나오는 것까지 막힘 = 진짜 앞뒤 다 막힘 → 기다림
            self.trapped += 1
            if self.trapped > 3:            # 시도 1·2·3 다 실패
                self.go("help")
            else:
                self.go("trapped")

    def go(self, state):
        if state == "back" and not self.plan_step():   # 지나온 길이 없음 (출발 직후 등) → 빼지 않고 멈춘 채 Nav2 에
            self.send(0.0, 0.0)
            self.get_logger().warn(f"WHISKER back: {self.plan_text}")
            self.state_pub.publish(String(data=json.dumps({"state": "back", "msg": self.plan_text, "try": self.trapped},
                                                          ensure_ascii=False)))
            state = "drive"
        self.state, self.state_t = state, time.time()
        msg = {"stop": "멈춤 (0.5 s)", "back": "빼기 — " + self.plan_text,
               "trapped": f"앞뒤 다 막힘 → {self.wait_s:.0f}초 대기 후 시도 {self.trapped}/3",
               "help": "구조 요청 — 3번 다 못 빠짐. 풀기: /whisker/resume", "drive": "빠짐 → Nav2 에 돌려줌"}[state]
        self.get_logger().warn(f"WHISKER {state}: {msg}")
        self.state_pub.publish(String(data=json.dumps({"state": state, "msg": msg, "try": self.trapped}, ensure_ascii=False)))
        if state == "help":
            where = self.map_pose()
            threading.Thread(target=self.ntfy, daemon=True, args=("robot: WHISKER 구조 요청",
                      f"앞뒤가 막혀 3번 빠져나오지 못함. 위치 {where[1]:.2f}, {where[2]:.2f} ({where[0]})" if where else
                      "앞뒤가 막혀 3번 빠져나오지 못함")).start()

    def plan_step(self):
        """빼기 계획. 처음·시도 1: 그대로 / 시도 2: 덜 박힌 쪽으로 15° 먼저 / 시도 3: 반대로 15° 먼저"""
        x0, z0 = self.last_cmd
        rotating = abs(x0) < 0.01 and abs(z0) > 0.05
        pre = 0.0
        if self.trapped == 2:
            pre = -self.back_turn if self.left_hits > self.right_hits else self.back_turn
        elif self.trapped >= 3:
            pre = self.back_turn if self.left_hits > self.right_hits else -self.back_turn
        dist = self.soft_dist if self.contact_kind in ("slow", "blocked") else self.back_dist   # 밟은 건(roll·pitch) 10 cm
        if rotating:   # 돌다가 닿음 → 돌던 반대로 15°
            self.step = {"kind": "rot", "sign": -math.copysign(1, z0), "goal": self.back_turn, "pre": pre}
            main = f"{'왼쪽' if self.step['sign'] > 0 else '오른쪽'}으로 {math.degrees(self.back_turn):.0f}° 되돌리기"
        else:          # 가다가 닿음 → 왔던 길로, 온 만큼만 (최대 10 cm, 부드러운 건 5 cm)
            sign = -1.0 if math.cos(self.contact_ang) > 0 else 1.0
            came = self.trail["len"] if self.trail["sign"] == -sign else 0.0      # 그 방향으로 지나온 거리
            dist = min(dist, came)
            if dist < 0.02:
                self.step = None
                self.plan_text = f"지나온 길 없음 ({came * 100:.0f} cm) → 빼지 않고 멈춤, Nav2 에 돌려줌 (표시는 남김)"
                return False
            self.step = {"kind": "lin", "sign": sign, "goal": dist, "pre": pre}
            main = f"{'뒤' if sign < 0 else '앞으'}로 {dist * 100:.0f} cm (지나온 {came * 100:.0f} cm 안에서)"
        self.step["start"] = self.odom
        self.step["phase"] = "pre" if pre else "main"
        head = f"시도 {self.trapped}: " if self.trapped else ""
        turn = f"{'왼쪽' if pre > 0 else '오른쪽'} {math.degrees(abs(pre)):.0f}° 돈 뒤 " if pre else ""
        self.plan_text = f"{head}{turn}{main} ({self.contact_feel})"
        return True

    def backed(self):
        st = self.step
        if not st or st.get("phase") != "main" or not self.odom or not st.get("start"):
            return 0.0
        return math.hypot(self.odom[0] - st["start"][0], self.odom[1] - st["start"][1])

    def backed_frac(self):
        """지금 빼기에서 목표의 몇 % 를 왔나 (직진만. 회전·준비 회전 중엔 1 = 무시 안 함)"""
        st = self.step
        if not st or st.get("phase") != "main" or st["kind"] != "lin" or not self.odom or not st.get("start"):
            return 1.0
        return math.hypot(self.odom[0] - st["start"][0], self.odom[1] - st["start"][1]) / max(st["goal"], 1e-3)

    def tick(self):
        if self.state == "drive":
            return
        now, t = time.time(), time.time() - self.state_t
        x = z = 0.0
        if self.state == "stop":
            if t > 0.5:
                self.go("back")
        elif self.state == "trapped":
            if t > self.wait_s:
                self.go("back")
        elif self.state == "back":
            st = self.step
            if not self.odom or not st["start"]:
                self.go("drive")
                return
            x0, y0, yaw0 = st["start"]
            x1, y1, yaw1 = self.odom
            if st["phase"] == "pre":
                turned = abs(math.atan2(math.sin(yaw1 - yaw0), math.cos(yaw1 - yaw0)))
                if turned >= abs(st["pre"]):
                    st["phase"], st["start"] = "main", self.odom
                    x0, y0, yaw0 = self.odom
                else:
                    z = math.copysign(TURN_SPEED, st["pre"])
            if st["phase"] == "main":
                if st["kind"] == "lin":
                    done = math.hypot(x1 - x0, y1 - y0) >= st["goal"]
                    x = 0.0 if done else st["sign"] * BACK_SPEED
                    # 방향 유지: 시작 방향에서 틀어진 만큼 살짝 되돌림 (11-06: 좌우 바퀴가 따로 풀려 비스듬히 빠짐)
                    err = math.atan2(math.sin(yaw0 - yaw1), math.cos(yaw0 - yaw1))
                    z = 0.0 if done else max(-0.12, min(0.12, 1.5 * err))
                else:
                    done = abs(math.atan2(math.sin(yaw1 - yaw0), math.cos(yaw1 - yaw0))) >= st["goal"]
                    z = 0.0 if done else st["sign"] * TURN_SPEED
                if done:
                    self.trapped = 0
                    self.send(0.0, 0.0)
                    self.go("drive")
                    return
            if t > STEP_TIMEOUT:   # 안 움직임 (헛돎) → 갇힌 걸로
                self.trapped += 1
                self.go("help" if self.trapped > 3 else "trapped")
                return
        self.send(x, z)

    def send(self, x, z):
        m = TwistStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = "base_link"
        m.twist.linear.x, m.twist.angular.z = float(x), float(z)
        self.cmd_pub.publish(m)

    def on_resume(self, _):   # 사람이 풀어줌 / whisker_lab 이 Ctrl+C 로 끝낼 때
        self.trapped = 0
        self.send(0.0, 0.0)
        self.go("drive")

    # ------------------------------------------------------------------ 기억
    def map_pose(self):
        if not self.odom_hist:
            return None
        _, ox, oy, oyaw = self.odom_hist[-1]
        if not self.map_fix:
            return "odom", ox, oy, oyaw
        ts, mx, my, myaw = self.map_fix
        old = min(self.odom_hist, key=lambda r: abs(r[0] - ts))
        dx, dy = ox - old[1], oy - old[2]
        c, s = math.cos(-old[3]), math.sin(-old[3])
        lx, ly = c * dx - s * dy, s * dx + c * dy
        c, s = math.cos(myaw), math.sin(myaw)
        return "map", mx + c * lx - s * ly, my + s * lx + c * ly, myaw + (oyaw - old[3])

    def mark(self, ang, ttl=None, half=MARK_HALF):
        """충격 방향의 몸 둘레 점 + 3 cm 바깥에, 방향과 직각으로 점 3개 (~10 cm)"""
        pose = self.map_pose()
        if not pose or pose[0] != "map":
            return False
        _, x, y, yaw = pose
        ca, sa = math.cos(ang), math.sin(ang)
        r = min(HALF_L / abs(ca) if abs(ca) > 1e-6 else 9, HALF_W / abs(sa) if abs(sa) > 1e-6 else 9) + MARK_GAP
        n = max(1, round(half / 0.05))           # 5 cm 간격 (지도 칸)
        for off in [half * i / n for i in range(-n, n + 1)]:
            bx, by = r * ca - off * sa, r * sa + off * ca            # 몸 기준
            self.points.append((x + math.cos(yaw) * bx - math.sin(yaw) * by,
                                y + math.sin(yaw) * bx + math.cos(yaw) * by, time.time() + ttl if ttl else None))
        self.contacts += 1
        self.publish_cloud()
        return True

    def record(self, ev):
        pose = self.map_pose()
        if pose:
            ev.update(frame=pose[0], x=round(pose[1], 3), y=round(pose[2], 3), yaw=round(pose[3], 3))
        ev["t"] = round(time.time(), 2)
        with open(self.log_path, "a") as f:
            f.write(json.dumps(ev, ensure_ascii=False) + "\n")
        self.get_logger().warn("SENSE " + json.dumps(ev, ensure_ascii=False))
        self.sense_pub.publish(String(data=json.dumps(ev, ensure_ascii=False)))

    def on_clear(self, _):
        self.points.clear()
        self.contacts = 0
        self.get_logger().warn("WHISKER clear: 표시 전부 지움")
        self.publish_cloud()

    def publish_cloud(self):
        now = time.time()
        self.points = [p for p in self.points if p[2] is None or p[2] > now]   # 부드러운 표시는 만료되면 사라짐
        self.cloud_pub.publish(self.cloud([(x, y) for x, y, _ in self.points]))
        self.soft_pub.publish(self.cloud([(x, y) for x, y, exp in self.points if exp is not None]))

    def cloud(self, pts):
        m = PointCloud2()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = "map"
        m.height, m.width = 1, len(pts)
        m.fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1) for i, n in enumerate("xyz")]
        m.is_bigendian, m.point_step, m.is_dense = False, 12, True
        m.row_step = 12 * len(pts)
        m.data = b"".join(struct.pack("<fff", px, py, MARK_Z) for px, py in pts)
        return m

    @staticmethod
    def ntfy(title, body):
        env = Path.home() / ".scr.env"
        topic = None
        if env.exists():
            for ln in env.read_text().splitlines():
                if ln.startswith("NTFY_TOPIC="):
                    topic = ln.split("=", 1)[1].strip()
        if not topic:
            return
        try:
            urllib.request.urlopen(urllib.request.Request("https://ntfy.sh/" + topic, data=body.encode(), headers={
                "Title": title, "Priority": "high", "Tags": "warning"}), timeout=10)
        except Exception:
            pass


def main():
    rclpy.init()
    n = Whisker()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
