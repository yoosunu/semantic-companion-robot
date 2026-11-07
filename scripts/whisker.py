#!/usr/bin/env python3
"""Whisker (수염) v9 — 기어다니는 로봇의 촉각. 11-07: 감각 셋, 탈출 하나 (상황별 예외를 걷어냄).

  감각 — 출발·톡톡·곡선·빼기 어디서나 같은 규칙
    부딪힘     실제로 굴러가다 가속 튐 → 0.3 s 안에 바퀴가 실제로 멈췄나 확인
               (Jasper: "움직이다 박는 것"과 "서 있다 출발하는 흔들림"은 다르다)
    (뺀 것)    카펫 기울기(가속도 = 중력 방향): 11-07 01:54 실주행에서 헛것 5번, 진짜 0번. 가속도계는 기울기와 우리 가속을
               구별 못 함 → 톡톡(0.5 s 밀고 멈춤) 중 5.3°, 곡선·후진 중 1.6~1.9° 로 방 가운데 분홍 선. efference 원칙과 충돌 → 뺌
    들림       0.3 s 동안 자이로로 실제 기운 각도 1.8° 이상 = 선·문턱을 밟고 올라탐 → 바로 되감기, 일직선 영구 표시
               11-07 실측: 평소 ≤ 1.5° / 굵은 선 2.0~3.9° / 화장실 문턱 4.9°. 얇은 카펫은 못 잡음 (ToF 오면 다시)
    기운 채    큰 흔들림이 잦아든 때(자이로 0.5 s 평균 < 3°/s) 중력 방향이 출발 전 자세보다 4° 이상 = 문턱·경사에 올라섬 → 되감기
               (Jasper 11-08 "캡슐"). 실측(명령 중): 평소 ≤ 3.2° / 선 ≤ 2.8° / 올라탄 문턱 5.6~10°
    안 움직임  움직이라는 명령이 1.5 s 이어졌는데 LiDAR 스캔 맞추기로 잰 이동이 1.5 cm 미만이고 자이로도 3° 도 안 돎
               = 헛바퀴·막힘·손으로 막음. 바퀴 거리계는 안 믿음 (11-07 00:36: 헛바퀴를 거리계가 "움직임"으로 봐서 놓침)
  탈출 하나    Nav2 를 끊고 지나온 길을 되감음 (Jasper 11-07). 들어올 때 기록한 자이로 방향을 거꾸로 따라가며 후진.
               헛도는 쪽이 있어 차체가 틀어지면 방향 보정이 그쪽 바퀴를 세게, 반대쪽을 약하게 → 들어온 곡선 그대로 나옴.
               최대 15 cm, 들어온 방향에서 30° 까지 → 끝나면 Nav2 에 돌려줌. 지나온 길이 없으면(출발 직후) 표시만 하고 돌려줌
  갇힘         빼다가 또 막힘 → 2 cm 이상 뺐으면 방금 뺀 길로 반대로 최대 5 cm → 그래도 막히면 10초 대기 → 다시
               (시도 1: 남은 되감기, 2: 15° 돌고 빼기, 3: 반대로 15°) → 3번 실패 = 구조 요청(ntfy) + 정지
               풀기: ros2 topic pub --once /whisker/resume std_msgs/msg/Empty
  표시         부딪힘 = 점 10 cm 영구 / 들림 = 선 30 cm 영구 /
               안 움직임 = 선 30 cm 1분, 같은 곳(30 cm 안)에서 또 = 영구 ("헛돌면 못 가는 구역")
               /whisker/cloud → costmap whisker_layer (LiDAR 가 못 지움). 지우기: ros2 topic pub --once /whisker/clear std_msgs/msg/Empty
  힘 제한      브리지 v8.1: PWM ≤ 40 직진 / ≤ 55 회전 — 감지가 늦어도 짓누르지 않게
  기록         ~/scr_logs/whisker/<날짜>.jsonl, 로그 줄 "SENSE {...}" (run_report 가 셈)

브리지는 얇게 (v8): /imu, /motor_out 을 내고, /whisker/cmd_vel 이 오는 동안은 Nav2 를 무시한다. 판단은 전부 여기.
문턱값은 ROS 파라미터 → 재빌드 없이 `ros2 param set /whisker bump_g 0.05`. 근거: docs/calibration/imu-touch-2026-11-06.md
"""
import collections
import json
import math
import struct
import threading
import time
import urllib.request
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, LaserScan, PointCloud2, PointField
from std_msgs.msg import Empty, Float32MultiArray, String

G = 9.81
HALF_L, HALF_W = 0.13, 0.12   # m, 차체 반길이·반폭 (footprint)
MARK_GAP = 0.03               # m, 몸 바깥으로
MARK_HALF = 0.05              # m, 점 표시 반폭 → 폭 ~10 cm (의자 다리)
LINE_HALF = 0.15              # m, 선 표시 반폭 → 폭 ~30 cm (카펫 가장자리·턱은 길다. 11-07 00:36: 10 cm 옆으로 다시 올라탐)
MARK_Z = 0.10                 # m, 장애물 높이 (costmap 높이 필터 안)
SELF_MASK = 0.4               # s, 우리가 모터를 바꾼 뒤 IMU 무시 (efference copy)
MOVING_V, MOVING_W = 0.025, 0.10   # 직전 0.5 s 실제로 이만큼 움직였으면 = 굴러가는 중 (부딪힘 조건)
CONFIRM_S, CONFIRM_STALL = 0.3, 0.5   # 부딪힘 의심 → 확인 창 / 바퀴 멈춤(진척 < 명령의 50%)
NOMO_BACK_S = 2.5             # s, 빼는 중의 안 움직임 창
NOMO_S = 1.5                  # s, 안 움직임 판단 창 (11-07 체중계: 충격 없이 막히고 바퀴는 헛돎 → LiDAR 만 봄. 2.5 → 1.5 s)
LIFT_DEG = 1.8                # °, 0.3 s 동안 자이로로 실제 기운 각도(앞뒤+좌우) = 뭔가를 밟고 올라탐
LIFT_WIN = 0.3                #   11-07 실측: 평소 18개(출발·정지·회전·카펫·상자·손·발·밀기) ≤ 1.5° /
                              #   굵은 선 2.0°(0.05)·3.9°(0.08), 화장실 문턱 4.9°, 체중계에 막힘 1.5°(안 올라탐)
SLOPE_DEG = 4.0               # °, 흔들림이 잦아든 때(자이로 0.5 s 평균이 조용) 중력 방향이 출발 전 자세보다 이만큼 기운 채 = 문턱·경사에 올라섬
SLOPE_QUIET, SLOPE_WIN = 3.0, 0.5   # °/s, s — "조용" = 이 창 동안 자이로 크기의 "평균"이 이 아래 (Jasper 11-08 캡슐: 흔들림은 빨리 왔다
                              #   돌아가고, 올라선 건 기운 채 남는다). 11-08 00:43 실기: "최댓값 1.5 미만"은 바퀴가 도는 동안 한 번도 안 옴
                              #   → 평균 3.0. 실측(명령 중만): 평소 ≤ 3.2°(손가락 밟기) / 선 ≤ 2.8° / 올라탄 문턱 5.6·7.7·10.0°
SLOPE_WATCH = 2.0             # °, 이 이상이면 값만 로그 (톡톡·곡선 주행 기준점 잡기용)
LIFT_WATCH = 1.0              # °, 이 이상이면 값만 로그 (Nav2 곡선·톡톡에서 얼마나 나오나 — 기록 18개엔 없던 주행)
NOMO_MIN_LIN, NOMO_MIN_ROT = 0.03, 0.2   # 창 동안 명령의 합(부호 포함): 3 cm·0.2 rad 이상 가라고 했어야 판단 (창 1.5 s 라 느린 0.03 m/s 도 들어오게)
NOMO_TURN = 0.05              # rad, 자이로로 3° 이상 돌았으면 움직인 것
NOMO_MOVE = 0.015             # m, 2.5 s 동안 LiDAR 로 잰 이동이 이 미만 = 안 움직임 (느린 건 막힘이 아님)
NOMO_SHARP = 0.003            # m, 스캔 맞추기 답이 ±3 cm 옆보다 평균 잔차가 이만큼은 작아야 믿음 (특징 없는 곳이면 판단 안 함)
NOMO_RANGE = 10.0             # m
NOMO_PTS = 240                # 스캔 한 장에서 쓸 점 수
REPEAT_R = 0.30               # m, 안 움직임이 같은 곳에서 또 → 영구
BACK_SPEED, TURN_SPEED = 0.05, 0.15   # m/s, rad/s — 바닥선 바로 위, 연속으로 잔잔하게
HEAD_K, HEAD_MAX = 1.5, 0.60  # 방향 보정. z 0.5 면 바퀴 −0.10 / 0.00 = 한쪽만 굴림 (헛도는 쪽을 세게, 잡힌 쪽은 멈춤)
HEAD_ROT = math.radians(10)   # 되감기 끝에 방향이 이보다 남으면 제자리에서 맞춤 (돌던 중 닿은 것 되돌리기)
REPLAY_MAX = 0.15             # m, 되감기 최대 거리
REPLAY_ROT = math.radians(30)  # 되감기는 들어온 방향에서 30° 까지만 거슬러 감
HIST_S = 10.0                 # s, 지나온 길 기록 길이
IDLE_RESET = 1.0              # s, 이만큼 서 있으면 지나온 길 새로 시작
REBOUND_MAX = 0.05            # m
STEP_TIMEOUT = 12.0           # s, 빼기 한 번 최대
SOFT_TTL = 60.0               # s, 부드러운 것(발·손일 수도) / 처음 막힌 곳 표시는 1분 뒤 사라짐


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def side_name(ang):
    a = math.degrees(wrap(ang))
    for lo, hi, n in ((-45, 45, "front"), (45, 135, "left"), (-135, -45, "right")):
        if lo <= a < hi:
            return n
    return "rear"


class Whisker(Node):
    def __init__(self):
        super().__init__("whisker")
        p = self.declare_parameter
        self.bump_g = p("bump_g", 0.06).value          # 주행 중 수평 가속 튐 (상자 0.09, 주행 잡음 0.02)
        self.touch_g = p("touch_g", 0.025).value       # 정지 중 건드림 (톡 0.04)
        self.rock_dps = p("rock_dps", 4.0).value       # 정지 중 기우뚱 (톡 6)
        self.back_turn = math.radians(p("back_turn_deg", 15.0).value)
        self.wait_s = p("trapped_wait", 10.0).value
        self.ax_forward = p("ax_forward", 1.0).value   # IMU 방향은 정지 중 참고용 (미확정)
        self.ay_left = p("ay_left", 1.0).value
        self.react = p("react", True).value            # False = 느끼고 기록만
        self.lift_deg = p("lift_deg", LIFT_DEG).value   # 시험용으로 끌 때: -p lift_deg:=99.0
        self.slope_deg = p("slope_deg", SLOPE_DEG).value
        self.laser_yaw = math.radians(p("laser_yaw_deg", 180.0).value)   # URDF laser_yaw (10-29 실측: 라이다 0° = 로봇 뒤)

        # 감각
        self.base = None                     # [ax, ay, gx, gy] 평소 값
        self.motor = [0.0] * 5               # /motor_out [pwm_l, pwm_r, cmd_x, cmd_z, whisker_active]
        self.motor_hist = collections.deque()          # (t, pwm_l, pwm_r, cmd_x, cmd_z) 최근 0.4 s
        self.cmd_hist = collections.deque()            # (t, cmd_x, cmd_z) 최근 NOMO_S + 1 s
        self.scans = collections.deque()               # (t, 스캔 점들 (x, y) 몸 기준)
        self.yaw_hist = collections.deque()            # (t, 자이로 방향)
        self.lift_hist = collections.deque()  # (t, 좌우 °/s, 앞뒤 °/s, dt) 최근 0.3 s
        self.lift_watch_t = 0.0
        self.lift_t = 0.0
        self.grav = collections.deque()      # (t, ax, ay, az, 자이로 크기) 최근 0.5 s — 기운 채 감각
        self.grav_ref = None                 # 출발 전(멈춰 있고 조용할 때) 중력 방향
        self.slope_t = 0.0
        self.slope_watch_t = 0.0
        self.diag = {"imu": 0, "motor": 0, "deg": 0.0, "asked": 0, "dt0": 0, "nomo": {}, "slope": 0.0}   # 5 s 마다 상태 한 줄 (11-07 23:48: 아무것도 못 느낀 원인 찾기)
        self.pending = None                  # 의심 중인 사건
        self.prev_imu_t = None
        self.last_self = 0.0
        self.last_event = 0.0
        self.nomo_t = 0.0
        self.nomo_check = 0.0
        self.stuck_spots = []                # 안 움직임이 났던 곳 (지도)
        # 지나온 길 (Nav2 가 운전하는 동안): (t, x, y, yaw), 한 방향(앞/뒤)으로만 이어짐
        self.path = collections.deque()
        self.path_sign = 0
        self.idle_since = None
        # 위치
        self.odom = None
        self.odom_hist = collections.deque(maxlen=20 * 180)   # (stamp, x, y, yaw) — 지도 위치 계산용
        self.map_fix = None
        # 반응
        self.state = "drive"                 # drive / stop / back / trapped / help
        self.state_t = 0.0
        self.esc = None                      # 되감기 원본 {"dir", "wps": [(s, yaw)], "len", "done"}
        self.step = None                     # 지금 빼기 {"dir", "wps", "len", "pre", "phase", "start", "hold", "rebound"}
        self.contact_kind = "bump"
        self.contact_feel = "hard"
        self.plan_text = ""
        self.trapped = 0
        self.left_hits, self.right_hits = 0, 0
        # 기억
        self.points = []                     # (x, y, 만료 시각 또는 None=영구)
        self.contacts = 0

        self.create_subscription(Imu, "imu", self.on_imu, 50)
        self.create_subscription(Float32MultiArray, "motor_out", self.on_motor, 50)
        self.create_subscription(Odometry, "odom", self.on_odom, 20)
        self.create_subscription(LaserScan, "scan", self.on_scan, qos_profile_sensor_data)
        for t in ("pose", "amcl_pose"):
            self.create_subscription(PoseWithCovarianceStamped, t, self.on_map_pose, 10)
        self.create_subscription(Empty, "whisker/clear", self.on_clear, 1)
        self.create_subscription(Empty, "whisker/resume", self.on_resume, 1)
        self.cmd_pub = self.create_publisher(TwistStamped, "whisker/cmd_vel", 10)
        self.cloud_pub = self.create_publisher(PointCloud2, "whisker/cloud", 1)
        self.soft_pub = self.create_publisher(PointCloud2, "whisker/soft_cloud", 1)   # RViz 색 구분용 (임시 표시만)
        self.sense_pub = self.create_publisher(String, "sense", 10)
        self.state_pub = self.create_publisher(String, "whisker/state", 10)
        self.create_timer(0.05, self.tick)
        self.create_timer(1.0, self.publish_cloud)
        self.create_timer(5.0, self.say_diag)
        d = Path.home() / "scr_logs" / "whisker"
        d.mkdir(parents=True, exist_ok=True)
        self.log_path = d / f"{time.strftime('%Y-%m-%d')}.jsonl"
        self.get_logger().info(f"whisker v9 ready (react={self.react}) → log {self.log_path}")

    # ------------------------------------------------------------------ 입력
    def say_diag(self):
        g = self.diag
        self.get_logger().info(f"DIAG 5s: imu {g['imu']} · motor {g['motor']} · 명령중 {g['asked']} · dt=0 {g['dt0']} · "
                               f"최대 들림 {g['deg']:.1f}° · 최대 기운채 {g['slope']:.1f}° · 안움직임 {g['nomo'] or '-'} · cmd {self.motor[2]:.3f} {self.motor[3]:.2f} · {self.state}")
        self.diag = {"imu": 0, "motor": 0, "deg": 0.0, "asked": 0, "dt0": 0, "nomo": {}, "slope": 0.0}

    def on_motor(self, m):
        self.diag["motor"] += 1
        d = (list(m.data) + [0.0] * 5)[:5]
        self.motor = d
        now = time.time()
        # efference copy: 0.4 s 전과 비교해 우리가 모터를 바꿨으면 그 흔들림은 우리 것
        self.motor_hist.append((now, d[0], d[1], d[2], d[3]))
        while self.motor_hist and now - self.motor_hist[0][0] > SELF_MASK:
            self.motor_hist.popleft()
        o = self.motor_hist[0]
        if abs(d[0] - o[1]) > 8 or abs(d[1] - o[2]) > 8 or abs(d[2] - o[3]) > 0.02 or abs(d[3] - o[4]) > 0.05:
            self.last_self = max(self.last_self, now)
        self.cmd_hist.append((now, d[2], d[3]))
        while self.cmd_hist and now - self.cmd_hist[0][0] > NOMO_BACK_S + 1.0:
            self.cmd_hist.popleft()

    def on_odom(self, m):
        st, pp = m.header.stamp, m.pose.pose
        self.odom = (pp.position.x, pp.position.y, yaw_of(pp.orientation))
        self.odom_hist.append((st.sec + st.nanosec * 1e-9, *self.odom))
        now = time.time()
        self.yaw_hist.append((now, self.odom[2]))
        while self.yaw_hist and now - self.yaw_hist[0][0] > NOMO_BACK_S + 1.0:
            self.yaw_hist.popleft()
        self.update_path(now)

    def update_path(self, now):
        """Nav2 가 운전하는 동안 지나온 길. 앞/뒤 방향이 바뀌거나 1 s 넘게 서 있으면 새로 시작"""
        if self.state != "drive":
            return
        cx, cz = self.motor[2], self.motor[3]
        sign = 1 if cx > 0.01 else (-1 if cx < -0.01 else 0)
        if sign == 0 and abs(cz) < 0.05:
            self.idle_since = self.idle_since or now
            if now - self.idle_since > IDLE_RESET:
                self.path.clear()
                self.path_sign = 0
            return
        self.idle_since = None
        if sign and self.path_sign and sign != self.path_sign:
            self.path.clear()
        if sign:
            self.path_sign = sign
        self.path.append((now, *self.odom))
        while self.path and now - self.path[0][0] > HIST_S:
            self.path.popleft()

    def moving_now(self):
        """직전 0.5 s 동안 실제로 움직였나 (거리계·자이로) — 부딪힘의 "굴러가던 중" 조건"""
        if len(self.odom_hist) < 3:
            return False
        t1, x1, y1, a1 = self.odom_hist[-1]
        old = next((r for r in reversed(self.odom_hist) if t1 - r[0] >= 0.5), None)
        if not old:
            return False
        dt = t1 - old[0]
        return (math.hypot(x1 - old[1], y1 - old[2]) / dt >= MOVING_V or abs(wrap(a1 - old[3])) / dt >= MOVING_W)

    def on_map_pose(self, m):
        st, pp = m.header.stamp, m.pose.pose
        self.map_fix = (st.sec + st.nanosec * 1e-9, pp.position.x, pp.position.y, yaw_of(pp.orientation))

    def on_scan(self, m):
        now = time.time()
        r = np.asarray(m.ranges, dtype=float)
        a = m.angle_min + np.arange(len(r)) * m.angle_increment + self.laser_yaw   # 몸 기준 (앞 = +x)
        ok = np.isfinite(r) & (r > max(m.range_min, 0.05)) & (r < NOMO_RANGE)
        pts = np.stack([r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok])], axis=1)
        if len(pts) > NOMO_PTS:
            pts = pts[:: int(math.ceil(len(pts) / NOMO_PTS))]
        self.scans.append((now, pts))
        while self.scans and now - self.scans[0][0] > NOMO_BACK_S + 1.0:
            self.scans.popleft()
        if now - self.nomo_check >= 0.5:
            self.nomo_check = now
            self.check_nomo(now)
            self.update_scan_backed(now)

    @staticmethod
    def scan_match(old, new, dyaw, reach):
        """옛 스캔을 몸 앞뒤로 밀어 보며 지금 스캔과 가장 잘 겹치는 거리 → (몇 cm 갔나, 그 답이 얼마나 분명한가 m).
        방향 차이는 자이로 값으로 먼저 맞춤. 지금 스캔은 이웃 점끼리 이은 선분으로 비교 (점↔점이면 빔 간격이 "움직임"처럼 보임).
        옆벽 점은 밀어도 벽 위를 미끄러질 뿐이라 판단에 안 끼고 앞·뒤의 것들이 정함 (거리 중앙값 비교는 옆벽에 속음).
        2 cm 간격으로 훑고 1 cm 로 다듬음 (Pi 부담: 0.5 s 마다 ~10 ms)"""
        c, s = math.cos(-dyaw), math.sin(-dyaw)
        if len(old) > 120:
            old = old[:: int(math.ceil(len(old) / 120))]
        old = np.stack([c * old[:, 0] - s * old[:, 1], s * old[:, 0] + c * old[:, 1]], axis=1)
        ux, uy = c, s                                   # 옛 몸 x 축을 지금 몸 기준으로
        a, b = new[:-1], new[1:]
        joined = np.hypot(*(b - a).T) < 0.15
        a = np.concatenate([a[joined], new])            # 이어진 선분 + 점 하나짜리(길이 0) 선분
        ab = np.concatenate([b[joined], new]) - a
        L2 = np.maximum((ab ** 2).sum(axis=1), 1e-12)
        cost = {}

        def at(i):
            if i not in cost:
                q = old - np.array([ux, uy]) * (i * 0.01)
                aq = q[:, None, :] - a[None, :, :]
                t = np.clip((aq * ab[None]).sum(axis=2) / L2[None], 0.0, 1.0)
                d = np.sqrt(((aq - t[..., None] * ab[None]) ** 2).sum(axis=2)).min(axis=1)
                cost[i] = float(np.minimum(d, 0.05).mean())
            return cost[i]
        n = int(round(reach / 0.01))
        best = min(range(-n - (n % 2), n + 1, 2), key=at)
        best = min((best - 1, best, best + 1), key=at)
        sharp = min(at(best - 3), at(best + 3)) - at(best)
        return best * 0.01, sharp

    def update_scan_backed(self, now):
        """빼는 동안 실제로 몇 cm 뺐나를 LiDAR 로 (거리계는 헛도는 바퀴만큼 부풀림 — 11-07 흉내 시험에서 드러남)"""
        st = self.step
        if self.state != "back" or not st or st["phase"] != "main" or st.get("scan0") is None or not self.scans:
            return
        shift, sharp = self.scan_match(st["scan0"], self.scans[-1][1], wrap(self.odom[2] - st["start"][2]),
                                       min(0.30, st["len"] + 0.08))
        if sharp >= NOMO_SHARP:                          # 분명한 답이 있을 때만 (특징 없는 곳이면 거리계)
            st["scan_s"], st["scan_t"] = abs(shift), now

    def nomo_why(self, now):
        """안 움직임을 판단하지 않는 이유 (없으면 None = 막힘). DIAG 에 셈.
        창: Nav2 주행 중 1.5 s (체중계처럼 막히면 빨리) / 빼는 중 2.5 s (한쪽이 헛돌며 느리게 나오는 중을 막힘으로 보지 않게)"""
        S = NOMO_S if self.state == "drive" else NOMO_BACK_S
        if self.state not in ("drive", "back") or now - self.nomo_t < S:
            return "상태·쉼"
        if self.state == "back" and now - self.state_t < S:   # 창이 빼기 안에 다 들어와야
            return "빼기 초반"
        if not self.cmd_hist or self.cmd_hist[0][0] > now - S + 0.2:
            return "명령 기록 짧음"
        win = [c for c in self.cmd_hist if c[0] >= now - S]
        if len(win) < 5 or any(abs(cx) < 0.01 and abs(cz) < 0.05 for _, cx, cz in win):
            return "명령 없음"                         # 중간에 "서라"가 있었음 = 판단 안 함
        lin = sum(c[1] * (n[0] - c[0]) for c, n in zip(win, win[1:]))
        rot = sum(c[2] * (n[0] - c[0]) for c, n in zip(win, win[1:]))
        if abs(lin) < NOMO_MIN_LIN and abs(rot) < NOMO_MIN_ROT:
            return "명령 적음"                         # 가라고 한 게 너무 적음 (제자리 떨림 등)
        yw = [(t, y) for t, y in self.yaw_hist if t >= now - S]
        if len(yw) < 5:
            return "자이로 기록 짧음"
        # 처음과 끝의 방향 차이 (11-07 23:57: 떨림의 절댓값을 다 더했더니 막혀서 떠는 것도 "3° 돎 = 움직임"이 될 수 있었음)
        turned = abs(wrap(yw[-1][1] - yw[0][1]))
        if turned >= NOMO_TURN:
            return "자이로 돎"                         # 자이로가 돎 = 움직임 (자이로는 헛바퀴에 안 속음)
        old = next((p for t, p in self.scans if t >= now - S), None)
        if old is None or len(self.scans) < 2 or len(old) < 30 or len(self.scans[-1][1]) < 30:
            return "스캔 없음"
        shift, sharp = self.scan_match(old, self.scans[-1][1], wrap(yw[-1][1] - yw[0][1]), min(0.25, abs(lin) + 0.05))
        # 기준 하나: 2.5 s 동안 LiDAR 로 1.5 cm 도 못 감 = 막힘. 11-07 02:35: "명령의 30%" 기준은 느린 후진 곡선
        #   (명령 12 cm, 실제 3 cm 움직임)을 막힘으로 오해 → 고스트. 진짜 막힘(발닦개)은 0~1 cm
        need = NOMO_MOVE
        if sharp < NOMO_SHARP:
            return "스캔 애매"                         # 특징 없는 곳 = 판단 못 함
        if abs(shift) >= need:
            return f"감 {abs(shift) * 100:.0f}cm"       # 실제로 갔음
        self.nomo_v = (lin, rot, turned, shift, sharp)
        return None

    def check_nomo(self, now):
        """안 움직임: 명령은 1.5 s 내내 "가라"였는데 세상(LiDAR)도 방향(자이로)도 그대로"""
        why = self.nomo_why(now)
        if why:
            self.diag["nomo"][why] = self.diag["nomo"].get(why, 0) + 1
            return
        self.diag["nomo"]["막힘"] = self.diag["nomo"].get("막힘", 0) + 1
        lin, rot, turned, shift, sharp = self.nomo_v
        self.nomo_t = now
        linear = abs(lin) >= NOMO_MIN_LIN
        ang = (0.0 if lin > 0 else math.pi) if linear else 0.0
        ev = {"ev": "stuck", "scan_cm": round(shift * 100, 1), "sharp_mm": round(sharp * 1000, 1),
              "turned_deg": round(math.degrees(turned), 1), "want_cm": round(abs(lin) * 100, 1),
              "want_deg": round(math.degrees(abs(rot))), "dir": side_name(ang) if linear else "turn",
              "src": "motion" if linear else "turn", "cmd_x": round(self.motor[2], 3), "cmd_z": round(self.motor[3], 2),
              "state": self.state}
        self.on_contact(ev, ang, cut=now - NOMO_S)       # 창 동안은 실제로 안 움직였으니 되감기에서 뺌

    def on_imu(self, m):
        now = time.time()
        a, w = m.linear_acceleration, m.angular_velocity
        s = [a.x / G, a.y / G, math.degrees(w.x), math.degrees(w.y)]
        if self.base is None:
            self.base = s[:]
            return
        d = [s[i] - self.base[i] for i in range(4)]
        horiz = math.hypot(d[0], d[1])
        # 시간은 IMU 메시지에 찍힌 시각으로 (11-07 23:42 실기: 처리 시각으로 쟀더니 스캔 맞추기 동안 밀린 IMU 가
        #   한꺼번에 처리돼 dt ≈ 0 → 기울기가 안 쌓여 들림을 전혀 못 느낌. 흉내 시험은 시간을 직접 넣어서 안 드러남)
        st = getattr(getattr(m, "header", None), "stamp", None)
        t_msg = st.sec + st.nanosec * 1e-9 if st is not None and st.sec else now
        dt = 0.05 if self.prev_imu_t is None else min(0.2, max(0.0, t_msg - self.prev_imu_t))
        self.prev_imu_t = t_msg
        if self.check_lift(t_msg, now, d[2], d[3], dt):
            return
        if self.check_slope(t_msg, now, a.x / G, a.y / G, a.z / G, math.hypot(d[2], d[3])):
            return
        if self.pending:                       # 부딪힘 의심 중: 창이 끝나면 결과(바퀴가 멈췄나)로 판정
            if now - self.pending["t"] >= CONFIRM_S:
                self.confirm()
            return
        motors_on = abs(self.motor[0]) > 1 or abs(self.motor[1]) > 1
        asked = abs(self.motor[2]) >= 0.01 or abs(self.motor[3]) >= 0.05
        settled = now - self.last_self > SELF_MASK
        if settled and horiz < self.touch_g and abs(d[2]) < self.rock_dps and abs(d[3]) < self.rock_dps:
            self.base = [self.base[i] + 0.02 * d[i] for i in range(4)]   # 평소 값은 조용할 때만 천천히
        elif settled and not motors_on and not asked and abs(d[2]) < 1.0 and abs(d[3]) < 1.0:
            # 멈춰 있고 자이로도 조용한데 가속만 다름 = 기운 자리에 서 있음 (문턱 위 등) → 평소 값이 그 자세를 따라감
            #   (11-07 23:42: 문턱 위에 선 채로 1초마다 touch 가 끝없이 뜸)
            self.base[0] += 0.1 * d[0]
            self.base[1] += 0.1 * d[1]
        if not settled or now - self.last_event < 1.0:
            return
        ev = None
        if motors_on and horiz > self.bump_g and self.moving_now():      # 움직이다 박음 (Jasper 11-07)
            ev = {"ev": "bump", "g": round(horiz, 3)}
        elif not motors_on and not asked and (horiz > self.touch_g or abs(d[2]) > self.rock_dps or abs(d[3]) > self.rock_dps):
            ev = {"ev": "touch", "g": round(horiz, 3), "gx": round(d[2], 1), "gy": round(d[3], 1)}
        if not ev:
            return
        self.last_event = now
        # 방향: 직진 성분이 있으면 가던 쪽 (확실 → 표시). 없으면 IMU 참고만 (11-06: 방향 부호를 못 믿음 → 표시 안 함)
        if abs(self.motor[2]) >= 0.01:
            ang, src = (0.0 if self.motor[2] > 0 else math.pi), "motion"
        else:
            bx, by = self.ax_forward * d[0], self.ay_left * d[1]
            ang, src = (math.atan2(-by, -bx) if horiz > 1e-4 else 0.0), "imu"
        ev.update(dir=side_name(ang), src=src, ax=round(d[0], 3), ay=round(d[1], 3),
                  cmd_x=round(self.motor[2], 3), cmd_z=round(self.motor[3], 2), state=self.state)
        if ev["ev"] == "touch":
            self.on_contact(ev, ang)
            return
        self.pending = {"ev": ev, "ang": ang, "t": now, "pos": self.odom, "cmd": (self.motor[2], self.motor[3])}

    def check_lift(self, t_msg, now, gx, gy, dt):
        """들림: 0.3 s 동안 자이로로 실제 기운 각도(앞뒤·좌우 합)가 1.8° 이상 = 선·문턱을 밟고 올라타는 중 → 바로 되감기.
        자이로는 실제 회전만 잼 (어제 뺀 가속도 기울기는 우리 가속에 속았음). 출발·정지 흔들림은 0.3 s 안에 +− 로 되돌아옴"""
        self.lift_hist.append((t_msg, gx, gy, dt))
        while self.lift_hist and t_msg - self.lift_hist[0][0] > LIFT_WIN:
            self.lift_hist.popleft()
        roll = sum(h[1] * h[3] for h in self.lift_hist)
        pitch = sum(h[2] * h[3] for h in self.lift_hist)
        deg = math.hypot(roll, pitch)
        asked = abs(self.motor[2]) >= 0.01 or abs(self.motor[3]) >= 0.05
        self.diag["imu"] += 1
        self.diag["asked"] += int(asked)
        self.diag["dt0"] += int(dt <= 1e-4)
        self.diag["deg"] = max(self.diag["deg"], deg)
        if deg >= LIFT_WATCH and asked and now - self.lift_watch_t > 1.0:
            self.lift_watch_t = now
            self.get_logger().info(f"LIFT watch {deg:.1f}° (앞뒤 {pitch:+.1f}, 좌우 {roll:+.1f}) "
                                   f"cmd {self.motor[2]:.3f} {self.motor[3]:.2f} {self.state}")
        if deg < self.lift_deg or not asked or self.state != "drive" or now - self.lift_t < 1.0:
            return False                       # (부딪힘 의심과는 따로 쉼 — 같이 막으면 선을 놓침, 11-07 기록 재생에서 드러남)
        self.lift_t = self.last_event = now
        self.lift_hist.clear()
        self.pending = None
        linear = abs(self.motor[2]) >= 0.01
        ang = (0.0 if self.motor[2] > 0 else math.pi) if linear else 0.0
        ev = {"ev": "lift", "deg": round(deg, 1), "pitch_deg": round(pitch, 1), "roll_deg": round(roll, 1),
              "dir": side_name(ang) if linear else "turn", "src": "motion" if linear else "turn",
              "cmd_x": round(self.motor[2], 3), "cmd_z": round(self.motor[3], 2), "state": self.state}
        self.on_contact(ev, ang)
        return True

    def check_slope(self, t_msg, now, ax, ay, az, gyro):
        """기운 채 (Jasper 11-08 "캡슐"): 자이로가 0.5 s 평균으로 조용할 때만 = 큰 흔들림이 잦아들었을 때만, 가속도(중력 방향)로 본 자세를
        출발 전 자세와 비교. 4° 이상 기운 채 = 문턱·경사에 올라섬 → 되감기. 오르막·내리막 똑같이 잡힘.
        가속도계는 우리 가속에 속지만(11-07 톡톡 5.3° 고스트), 그건 흔들림이 있는 동안이라 "조용할 때만" 보면 걸러짐"""
        self.grav.append((t_msg, ax, ay, az, gyro))
        while self.grav and t_msg - self.grav[0][0] > SLOPE_WIN:
            self.grav.popleft()
        if len(self.grav) < 5 or t_msg - self.grav[0][0] < SLOPE_WIN * 0.8 or \
                sum(g[4] for g in self.grav) / len(self.grav) > SLOPE_QUIET:
            return False                       # 아직 흔들리는 중
        v = np.median(np.array([g[1:4] for g in self.grav]), axis=0)
        v = v / max(float(np.linalg.norm(v)), 1e-6)
        asked = abs(self.motor[2]) >= 0.01 or abs(self.motor[3]) >= 0.05
        moving = asked or abs(self.motor[0]) > 1 or abs(self.motor[1]) > 1
        if self.grav_ref is None:
            if not moving:
                self.grav_ref = v
            return False
        tilt = math.degrees(math.acos(float(np.clip(np.dot(v, self.grav_ref), -1.0, 1.0))))
        self.diag["slope"] = max(self.diag["slope"], tilt)
        if not moving and tilt < self.slope_deg and self.state == "drive":
            self.grav_ref = self.grav_ref + 0.05 * (v - self.grav_ref)   # 멈춰 있을 때 평평한 자세를 따라감 (바닥마다 조금씩 다름)
            self.grav_ref = self.grav_ref / float(np.linalg.norm(self.grav_ref))
        if tilt >= SLOPE_WATCH and asked and now - self.slope_watch_t > 1.0:
            self.slope_watch_t = now
            self.get_logger().info(f"SLOPE watch {tilt:.1f}° cmd {self.motor[2]:.3f} {self.motor[3]:.2f} {self.state}")
        if tilt < self.slope_deg or not asked or self.state != "drive" or now - self.slope_t < 2.0:
            return False
        self.slope_t = self.last_event = now
        self.pending = None
        linear = abs(self.motor[2]) >= 0.01
        ang = (0.0 if self.motor[2] > 0 else math.pi) if linear else 0.0
        ev = {"ev": "slope", "deg": round(tilt, 1), "dir": side_name(ang) if linear else "turn",
              "src": "motion" if linear else "turn", "cmd_x": round(self.motor[2], 3), "cmd_z": round(self.motor[3], 2),
              "state": self.state}
        self.on_contact(ev, ang)
        return True

    def confirm(self):
        p, self.pending = self.pending, None
        ev, kind = p["ev"], p["ev"]["ev"]
        cx0, cz0 = p["cmd"]
        cx1, cz1 = self.motor[2], self.motor[3]
        if kind == "bump":
            still_asked = abs(cx1) >= 0.8 * abs(cx0) and abs(cz1) >= 0.8 * abs(cz0)
            if abs(cx0) >= 0.01:
                want = abs(cx0) * CONFIRM_S
                got = math.hypot(self.odom[0] - p["pos"][0], self.odom[1] - p["pos"][1]) if self.odom and p["pos"] else want
            else:
                want = abs(cz0) * CONFIRM_S
                got = abs(wrap(self.odom[2] - p["pos"][2])) if self.odom and p["pos"] else want
            ok = still_asked and want > 0.005 and got < CONFIRM_STALL * want
            ev["confirm"] = {"want": round(want, 3), "got": round(got, 3), "asked": still_asked}
        else:
            ok = False
        if ok:
            self.on_contact(ev, p["ang"], cut=p["t"])     # 부딪힌 뒤 0.3 s 는 안 움직였음
        else:
            ev["was"], ev["ev"], ev["feel"], ev["marked"] = kind, "ghost", "ghost", False
            self.record(ev)

    # ------------------------------------------------------------------ 반응
    def on_contact(self, ev, ang, cut=None):
        kind = ev["ev"]
        if self.state == "back" and kind in ("lift", "slope"):
            return                                   # 빼는 중 기울어짐 = 올라탔던 것에서 내려오는 중
        if self.state in ("stop", "trapped", "help") and kind != "touch":
            return
        half, ttl, feel = MARK_HALF, None, "hard"
        if kind in ("lift", "slope"):
            half = LINE_HALF                         # 올라탐 = 선·문턱 = 일직선, 영구 (Jasper 11-07)
        elif kind == "stuck":                        # 처음 1분, 같은 곳에서 또면 영구
            half = LINE_HALF
            pose = self.map_pose()
            again = bool(pose) and any(math.hypot(pose[1] - x, pose[2] - y) < REPEAT_R for x, y in self.stuck_spots)
            if pose and self.state != "back":
                self.stuck_spots.append((pose[1], pose[2]))
            ev["again"] = again
            ttl, feel = (None, "hard") if again else (SOFT_TTL, "soft")
        elif kind == "touch":
            feel = "touch"
        ev["feel"] = feel
        # 빼는 중 막힘은 표시 안 함: 지나온 길은 방금 지나와서 비어 있음 — 못 빠지는 건 미끄러짐 탓 (11-07 검토: 뒤에 영구 선이 그어져
        #   돌아갈 길을 스스로 막았음)
        backing = self.state == "back"
        ev["marked"] = kind != "touch" and ev.get("src") == "motion" and not backing and self.mark(ang, ttl, half)
        self.record(ev)
        if kind == "touch" or not self.react:
            return
        if ang > 0.3:
            self.left_hits += 1
        elif ang < -0.3:
            self.right_hits += 1
        if self.state == "drive":
            self.contact_kind, self.contact_feel = kind, feel
            self.trapped = 0
            self.esc = self.plan_replay(time.time() if cut is None else cut)
            self.path.clear()
            self.path_sign = 0
            self.go("stop" if kind == "bump" else "back")
        elif self.state == "back" and self.step and not self.step["rebound"] and self.backed() >= 0.02:
            # 빼다가 반대쪽에 막힘 → 기다리지 않고 방금 뺀 길로 반대로 (그 길은 방금 지나와서 비어 있음)
            done = self.backed()
            self.esc["done"] += done
            self.step = {"dir": -self.step["dir"], "wps": [(0.0, None)], "len": min(REBOUND_MAX, done), "pre": 0.0,
                         "phase": "main", "start": self.odom, "hold": self.odom[2], "rebound": True,
                         "scan0": self.last_scan()}
            self.plan_text = f"반대쪽도 막힘 → 방금 뺀 길로 {self.step['len'] * 100:.0f} cm"
            self.state_t = time.time()
            self.say_state("back", "빼기 — " + self.plan_text)
        elif self.state == "back":
            if self.step and not self.step["rebound"]:
                self.esc["done"] += self.backed()
            elif self.step:
                self.esc["done"] = max(0.0, self.esc["done"] - self.backed())
            self.trapped += 1
            self.go("help" if self.trapped > 3 else "trapped")

    def plan_replay(self, cut):
        """지나온 길(cut 이전)을 거꾸로: [(되감은 거리 s, 그 지점의 자이로 방향)], 최대 15 cm·30°"""
        pts = [p for p in self.path if p[0] <= cut]
        if not pts:
            return {"dir": 0, "wps": [(0.0, None)], "len": 0.0, "done": 0.0}
        yaw_end = pts[-1][3]
        wps, s = [(0.0, yaw_end)], 0.0
        for new, old in zip(reversed(pts), reversed(pts[:-1])):
            if abs(wrap(old[3] - yaw_end)) > REPLAY_ROT:
                break
            s += math.hypot(new[1] - old[1], new[2] - old[2])
            if s >= REPLAY_MAX:
                wps.append((REPLAY_MAX, old[3]))
                s = REPLAY_MAX
                break
            wps.append((s, old[3]))
        return {"dir": -self.path_sign, "wps": wps, "len": s if self.path_sign else 0.0, "done": 0.0}

    def start_step(self):
        """이번 빼기 계획. 처음·시도 1 = 남은 되감기 / 시도 2 = 덜 막힌 쪽으로 15° 돌고 / 시도 3 = 반대로 15°"""
        b = self.esc
        if not b or not self.odom:
            self.plan_text = "계획 없음 → Nav2 에 돌려줌"
            return False
        rem = max(0.0, b["len"] - b["done"])
        if self.trapped <= 1:
            wps = [(0.0, self.wp_yaw(b["wps"], b["done"], None))] + [(s - b["done"], y) for s, y in b["wps"] if s > b["done"]]
            turn = max(abs(wrap(y - self.odom[2])) for _, y in wps if y is not None) if any(y is not None for _, y in wps) else 0
            if rem < 0.02 and turn < HEAD_ROT:
                if self.trapped == 0:
                    self.plan_text = "지나온 길 없음 → 빼지 않고 멈춤, Nav2 에 돌려줌 (표시는 남김)"
                    return False
                wps, rem = [(0.0, None)], 0.05
            pre = 0.0
            curve = math.degrees(max(abs(wrap(y - wps[0][1])) for _, y in wps if y is not None)) if wps[0][1] is not None else 0
            main = f"되감기 {rem * 100:.0f} cm" + (f" (곡선 {curve:.0f}°)" if curve >= 5 else "")
        else:
            side = 1 if (self.trapped == 2) == (self.left_hits <= self.right_hits) else -1
            pre, wps, rem = side * self.back_turn, [(0.0, None)], max(rem, 0.05)
            main = f"{'왼쪽' if pre > 0 else '오른쪽'} {math.degrees(self.back_turn):.0f}° 돈 뒤 {rem * 100:.0f} cm"
        dirn = b["dir"] or -1
        self.step = {"dir": dirn, "wps": wps, "len": rem, "pre": pre, "phase": "pre" if pre else "main",
                     "start": self.odom, "hold": self.odom[2], "rebound": False, "scan0": self.last_scan()}
        head = f"시도 {self.trapped}: " if self.trapped else ""
        self.plan_text = f"{head}{'뒤' if dirn < 0 else '앞'}로 {main} ({self.contact_kind})"
        return True

    def last_scan(self):
        return self.scans[-1][1] if self.scans and time.time() - self.scans[-1][0] < 0.5 else None

    @staticmethod
    def wp_yaw(wps, s, hold):
        y = lambda v: hold if v is None else v
        if s <= wps[0][0]:
            return y(wps[0][1])
        for (s0, y0), (s1, y1) in zip(wps, wps[1:]):
            if s <= s1:
                a, b = y(y0), y(y1)
                if a is None or b is None:
                    return a if b is None else b
                return a + (s - s0) / max(s1 - s0, 1e-6) * wrap(b - a)
        return y(wps[-1][1])

    def go(self, state):
        if state == "back" and not self.start_step():
            self.send(0.0, 0.0)
            self.say_state("back", self.plan_text)
            state = "drive"
        self.state, self.state_t = state, time.time()
        msg = {"stop": "멈춤 (0.5 s)", "back": "빼기 — " + self.plan_text,
               "trapped": f"앞뒤 다 막힘 → {self.wait_s:.0f}초 대기 후 시도 {self.trapped}/3",
               "help": "구조 요청 — 3번 다 못 빠짐. 풀기: /whisker/resume", "drive": "Nav2 에 돌려줌"}[state]
        self.say_state(state, msg)
        if state == "help":
            where = self.map_pose()
            threading.Thread(target=self.ntfy, daemon=True, args=("robot: WHISKER 구조 요청",
                             f"3번 빠져나오지 못함. 위치 {where[1]:.2f}, {where[2]:.2f} ({where[0]})" if where else
                             "3번 빠져나오지 못함")).start()

    def say_state(self, state, msg):
        self.get_logger().warn(f"WHISKER {state}: {msg}")
        self.state_pub.publish(String(data=json.dumps({"state": state, "msg": msg, "try": self.trapped}, ensure_ascii=False)))

    def backed(self):
        """이번 빼기에서 실제로 간 거리: LiDAR 스캔 맞추기 값이 있으면 그것 (헛바퀴에 안 속음), 없으면 거리계"""
        st = self.step
        if not st or st["phase"] != "main" or not self.odom:
            return 0.0
        if st.get("scan_t") and time.time() - st["scan_t"] < 1.0:
            return st["scan_s"]
        return math.hypot(self.odom[0] - st["start"][0], self.odom[1] - st["start"][1])

    def tick(self):
        if self.state == "drive":
            return
        t = time.time() - self.state_t
        x = z = 0.0
        if self.state == "stop":
            if t > 0.5:
                self.go("back")
                return
        elif self.state == "trapped":
            if t > self.wait_s:
                self.go("back")
                return
        elif self.state == "back":
            st = self.step
            if not self.odom or not st:
                self.go("drive")
                return
            yaw = self.odom[2]
            if st["phase"] == "pre":
                if abs(wrap(yaw - st["start"][2])) >= abs(st["pre"]):
                    st.update(phase="main", start=self.odom, hold=yaw, scan0=self.last_scan(), scan_t=None)
                else:
                    z = math.copysign(TURN_SPEED, st["pre"])
            if st["phase"] == "main":
                s = self.backed()
                err = wrap(self.wp_yaw(st["wps"], s, st["hold"]) - yaw)
                if s < st["len"]:
                    # 되감기 = 들어올 때의 곡률(기록) 그대로 + 자이로로 본 실제 방향과의 차이만큼 보정.
                    #   헛도는 쪽이 있어 차체가 그쪽으로 돌면 보정이 그쪽 바퀴를 세게, 반대쪽을 약하게 (Jasper 11-07)
                    ahead = self.wp_yaw(st["wps"], s + 0.01, st["hold"])
                    ff = wrap(ahead - self.wp_yaw(st["wps"], s, st["hold"])) / 0.01 * BACK_SPEED
                    x = st["dir"] * BACK_SPEED
                    z = max(-HEAD_MAX, min(HEAD_MAX, ff + HEAD_K * err))
                elif abs(err) >= HEAD_ROT:        # 거리는 다 됨, 방향이 남음 (돌던 중 닿음 = 되돌리기) → 제자리에서 맞춤
                    z = math.copysign(TURN_SPEED, err)
                else:
                    if not st["rebound"]:
                        self.esc["done"] += s
                    self.trapped = 0
                    self.send(0.0, 0.0)
                    self.go("drive")
                    return
            if t > STEP_TIMEOUT:
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

    def on_resume(self, _):
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
        """닿은 방향의 몸 둘레 + 3 cm 바깥에, 방향과 직각으로 5 cm 간격 점들"""
        pose = self.map_pose()
        if not pose or pose[0] != "map":
            return False
        _, x, y, yaw = pose
        ca, sa = math.cos(ang), math.sin(ang)
        r = min(HALF_L / abs(ca) if abs(ca) > 1e-6 else 9, HALF_W / abs(sa) if abs(sa) > 1e-6 else 9) + MARK_GAP
        n = max(1, round(half / 0.05))
        for off in [half * i / n for i in range(-n, n + 1)]:
            bx, by = r * ca - off * sa, r * sa + off * ca
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
        self.stuck_spots.clear()
        self.contacts = 0
        self.get_logger().warn("WHISKER clear: 표시 전부 지움")
        self.publish_cloud()

    def publish_cloud(self):
        now = time.time()
        self.points = [p for p in self.points if p[2] is None or p[2] > now]
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
