#!/usr/bin/env python3
"""scr_explore — 우리 탐색기. "다음에 어디로 가 볼까"를 정하고, 못 가는 곳은 이유와 함께 기억한다. (11-06 밤, explore_lite 대체)

왜 직접: explore_lite 는 Whisker 를 모른다 → 분홍(닿아서 확인한 못 가는 곳) 너머 경계로 계속 보냄. 그리고 경계 중심이 1 cm 만
움직여도 새 목표로 봐서 포기를 못 함 (11-06 카펫에서 3분 넘게 삽질). 우리 판단을 넣는다:

  1. SLAM 지도에서 경계(아는 빈 칸 ↔ 모르는 칸)를 찾아 묶는다. 30 cm 미만 묶음은 무시
  2. Whisker 표시 40 cm 안의 경계는 처음부터 제외 (이유: whisker) — 닿아서 확인했으니까
  3. 점수 = 3 × 거리 − 1 × 크기 (가깝고 클수록 먼저) → 1등을 Nav2 목표로
  4. 포기 (지역 단위 40 cm 로 기억 — 중심이 움직여도 같은 곳):
       가는 중 Whisker 접촉 2번 → whisker / Nav2 가 길을 못 찾음 → no_path / 90 s 동안 5 cm 도 못 가까워짐 → no_progress /
       도착했는데 그 경계가 그대로 → unseen (LiDAR 로 볼 수 없는 곳)
     Whisker 가 맡으면 Nav2 는 쉼 (11-08 Jasper: "우리 로직이 작동할 땐 Nav2 에 맡기지 않고, 끝나면 다시 넘겨주기"):
       Whisker 가 멈춤·빼기·갇힘·구조 요청에 들어가면 목표를 취소(포기 아님) → Whisker 가 돌려주고 1 s 뒤 같은 목표를 다시 보냄.
       그동안 우리 시계(15 s 멈칫·45 s 진척)도 멈춤. 접촉 수는 이어서 셈. 더듬기도 Whisker 가 빼는 동안 시계를 멈춤.
       이유: 빼기·갇힘 대기(10 s)가 Nav2 의 진척 검사(20 s)·복구 동작·우리 타임아웃과 겹쳐 "안 막혔는데 포기"가 될 수 있음
     바퀴 주인 (11-10 Jasper: "바퀴 주인은 늘 하나, 막히면 다음 주인에게"): nav2 / crawl(우리가 플래너 길을 기어감) / whisker.
       바뀔 때마다 "바퀴 주인: A → B (이유)" 한 줄 → 멈춰 있으면 누가 왜 멈췄는지 로그에 있음.
       진척 = Nav2 가 알려주는 "길 따라 남은 거리" 또는 직선거리, 둘 중 하나라도 5 cm 줄면 진척 (11-08 거실: 돌아가는 길에서
       직선거리가 늘어 45 s 에 끊김 — F·P·B). 15 s 진척 없으면(Nav2 얼음 — D·E) Nav2 를 끄고 crawl 이 같은 목표로 이어받음.
       끼임: crawl 의 플래너가 지금 자리에서 길을 못 만들거나, Nav2 "실패"가 같은 자리에서 또 나면 = 경계 탓이 아니라 로봇이 끼었음
       (11-08 거실 문 앞: 같은 자리에서 경계 3곳 포기 — J·K·L) → 포기 대신 /whisker/escape (탈출 모듈), 같은 자리 2번까지
     단계 (11-08 밤): ① 관찰점에서 봄 → ② 경계에 가장 가까운 칸(비용 < 85)으로 Nav2 → ③ 더듬기: Nav2 주행(MPPI)을 끄고
       Nav2 플래너에게 경계까지 "길만" 받아 그 길을 0.07 m/s 로 따라감 (Whisker 켜짐 — 닿으면 Whisker 가 빼고, 돌려주면 그 자리에서
       길을 다시 받음, 2번이면 whisker 포기). 길 없음 → no_path, 길이 1.5 m 넘음 → far. 다 갔는데 그대로 → unseen
       (11-08 22:14: 처음엔 직진 → 0.8 m 떨어진 곳에서 LiDAR 가 이미 그린 벽을 들이받음. 길찾기는 플래너, 기어가기는 우리)
       ②에서 경계 80 cm 안인데 15 s 동안 5 cm 도 못 다가가면(MPPI 멈칫 = "어리바리") 기다리지 않고 바로 ③
       (11-08 21:15: 2단계 목표가 벽에 붙은 경계 칸 자체 → MPPI 가 46 s 동안 들어가지 않고 떨기만 → no_progress 포기 2곳)
     가는 중 그 경계가 지도에서 사라지면 (이미 보임) → 목표 바꿈
  5. 경계 없음 → 집(0,0)으로 → 지도 + 경계(포기 이유) 자동 저장 + 폰 알림. 3분 동안 10 cm 도 못 움직이면 STUCK 알림
  RViz (/scr_explore/markers): 파랑 = 남은 경계, 빨강 공 + 글자 = 포기(이유), 초록 큰 공 = 갈 관찰점, 작은 공 = 보려는 경계,
    노란 공 = 더듬어 보는 경계

  [Pi] tmux new -d -s explore "bash -ic 'python3 ~/semantic-companion-robot/scripts/scr_explore.py --name room; exec bash'"
  끄기: tmux send-keys -t explore C-c  (목표 취소 후 멈춤)
"""
import argparse
import collections
import functools
import json
import math
import subprocess
import threading
import time
import traceback
import urllib.parse
import urllib.request
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import Point, PoseStamped, PoseWithCovarianceStamped, TwistStamped
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import ColorRGBA, Empty, String
from visualization_msgs.msg import Marker, MarkerArray

MIN_FRONTIER = 0.30      # m, 이보다 작은 경계 묶음 무시
REGION = 0.40            # m, 같은 곳 / Whisker 표시 근처
W_DIST, W_SIZE = 3.0, 1.0
WHISKER_GIVEUP = 2       # 가는 중 접촉 이만큼이면 포기
NO_PROGRESS_S, PROGRESS_M = 45.0, 0.05   # 11-07: 90 s 는 카펫 표시 너머 목표에 98 s 버팀
STUCK_S, STUCK_M = 180.0, 0.10
REACH_COST = 99          # 전역 비용지도(0~100) 이 미만 = Nav2 플래너가 지나갈 수 있는 칸 (99 = 몸이 닿음). 11-08 23:38 거실: 75 는 플래너보다
                         #   엄격해서 30~36 cm 틈(비용 75~98)을 "못 감"으로 봄 → 다른 방 경계 9곳이 관찰점 없음(no_view)으로 한꺼번에 포기
ROTATE_COST = 78         # 비용 이 미만 = 몸 중심이 장애물에서 17.7 cm(몸 외접원) 넘게 떨어짐 = 제자리에서 돌 수 있는 칸.
                         #   11-10 Jasper: "돌 수 없는 곳엔 안 들어간다" — 폭 25~35 cm 틈은 직진으로 들어가지만 돌아 나올 수 없어 후진뿐
                         #   (후진 금지 + 팬틸트가 뒤 LiDAR 를 가림). SLAM 은 입구에서 보면 됨
APPROACH_COST = ROTATE_COST   # 2단계 "경계 옆 칸" (11-08 엔 85 = 벽에 더 붙기 절충 → 11-10 돌 수 있는 칸까지만)
VIEW_COST = 40           # 관찰점은 더 넉넉히 (벽·가구에서 떨어진 곳)
VIEW_MIN, VIEW_MAX = 0.20, 1.00   # m, 관찰점 ↔ 경계 거리
SETTLE_S = 7.0           # s, 관찰점 도착 후 SLAM 지도 갱신(5 s) 기다림 (11-07 02:25: 도착 즉시 unseen 으로 전부 포기)
NOVIEW_S = 30.0          # s, 갈 자리가 이만큼 계속 없을 때만 포기 (처음엔 지도가 작아서 잠깐 없을 수 있음)
CONTACTS = ("bump", "lift", "slope", "stuck", "turn_blocked")   # Whisker v9 의 감각 (ghost·touch 는 접촉 아님)
CANCEL_WAIT = 15.0       # s, 취소 보냈는데 Nav2 답이 없으면 우리가 정리
STATUS_S = 30.0          # s, 상태 한 줄 주기
STALL_S = 15.0           # s, 2단계(경계로 다가감)에서 이만큼 5 cm 도 못 다가가면 Nav2 멈칫 → 바로 더듬기 (45 s 기다리지 않음)
PROBE_V = 0.07           # m/s, 더듬기 속도 (0.05 아래는 브리지가 톡톡 모드로 바꿈)
PROBE_NEAR = 0.80        # m, 2단계 멈칫 → 더듬기는 경계에서 이만큼 안일 때만 (멀면 Nav2 에 계속 맡김 → 45 s 규칙)
PROBE_PATH_MAX = 2.5     # m, 플래너 길이 이보다 길면 더듬기가 아니라 이동 → far 포기 (11-08 23:38: 1.5 는 가구 뒤 경계에서 2번 걸림)
VIEW_NEAR = 0.50         # m, 1단계: 관찰점 이만큼 안에서 15 s 멈칫(MPPI 가 거의 0 명령) → 도착으로 봄 (거의 같은 자리에서 봄)
PROBE_LOOK = 0.12        # m, 길 따라가기: 이만큼 앞의 길 점을 봄
PROBE_S = 25.0           # s, 더듬기 시간 제한 = 이것 + 길 길이 / 0.05 m/s (Whisker 가 맡는 동안은 멈춤)
WHISKER_BUSY = ("stop", "back", "trapped", "help")   # Whisker 가 몰고 있는 상태 (pause = 밀림 1 s 정지는 짧아서 Nav2 그대로)
WEDGE_R, WEDGE_S = 0.15, 120.0   # m, s — 같은 자리(15 cm) 판단, 그 자리 기억 시간
WEDGE_ESC_MAX = 2        # 같은 자리 탈출 요청 최대 (그 뒤엔 진짜 길 없음으로 포기)
PROBE_QUIET = 1.5        # s, 더듬기 전 Nav2 가 조용해질 때까지 (취소 후 velocity_smoother 가 0 까지 감속)


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def ntfy(title, body, urgent=False):
    env = Path.home() / ".scr.env"
    topic = None
    if env.exists():
        for ln in env.read_text().splitlines():
            if ln.startswith("NTFY_TOPIC="):
                topic = ln.split("=", 1)[1].strip()
    if not topic:
        return
    # 제목은 URL 에 (11-08: HTTP 헤더는 한글을 못 담음 → "탐색기 오류" 알림이 조용히 실패했었음)
    url = "https://ntfy.sh/" + topic + "?title=" + urllib.parse.quote(title)
    try:
        urllib.request.urlopen(urllib.request.Request(url, data=body.encode(), headers={
            "Priority": "high" if urgent else "default", "Tags": "warning" if urgent else "robot"}), timeout=10)
    except Exception as e:
        print(f"[scr_explore] 알림 실패: {e}", flush=True)


def safe(fn):
    """콜백이 오류를 내도 탐색기가 죽지 않게: 전체 오류를 로그에 남기고, 처음 한 번은 폰으로 알림.
    (11-07 00:36: 90 s no_progress 취소 직후 소리 없이 멈춤 — 오류 내용이 tmux 화면에만 있다 사라짐)"""
    @functools.wraps(fn)
    def run(self, *a, **k):
        try:
            return fn(self, *a, **k)
        except Exception:
            tb = traceback.format_exc()
            self.get_logger().error(f"EXPLORE 오류 ({fn.__name__}) — 계속 돎\n{tb}")
            self.errors += 1
            if self.errors == 1:
                threading.Thread(target=ntfy, args=("robot: 탐색기 오류", f"{fn.__name__}: {tb.strip().splitlines()[-1]}", True),
                                 daemon=True).start()
    return run


class Explorer(Node):
    def __init__(self, name):
        super().__init__("scr_explore")
        self.name = name
        self.grid = None                         # OccupancyGrid (SLAM 지도 — 저장용)
        self.cmap = None                         # 전역 비용지도 (경계·관찰점은 이걸로: 팽창·Whisker 표시 포함)
        self.whisker = []                        # [(x, y)] Whisker 표시
        self.gave_up = []                        # [{"x","y","reason"}]
        self.viewed = []                         # 1단계: 관찰점에서 본 경계 (x, y)
        self.approached = []                     # 2단계: 경계 옆 싼 칸까지 갔거나 Nav2 가 멈칫한 경계 (x, y) → 다음은 더듬기
        self.reached = []                        # 3단계: 더듬기까지 해 본 경계 (x, y)
        self.probe = None                        # 더듬기 중: {"x","y","t0","start","contacts","going","quiet_until","hold"}
        self.wstate = "drive"                    # Whisker 상태 (drive 일 때만 더듬기 명령을 냄)
        self.wstate_t = 0.0                      # Whisker 상태가 바뀐 시각
        self.paused = None                       # Whisker 가 맡는 동안 쉬는 목표 (끝나면 다시 보냄)
        self.home_h, self.home_paused = None, False
        self.owner = "none"                      # 바퀴 주인: none / nav2 / crawl / whisker (11-10)
        self.fails = []                          # Nav2·플래너 "길 없음"이 난 로봇 자리 [(t, x, y, 경계 x, 경계 y)]
        self.escapes = []                        # 탈출 요청한 자리 [(t, x, y)]
        # 11-08: 예전엔 관찰점 도착도 reached 에 넣어 "가 봤다"로 기억 → 통로에 들어가 보지도 않고 unseen (11시)
        self.frontiers = []                      # 마지막으로 찾은 [{"x","y","size","cells":[(x,y)]}]
        self.goal = None                         # {"x","y","t0","best","best_t","contacts","handle"}
        self.phase = "explore"                   # explore → home → done
        self.odom_hist = collections.deque(maxlen=20 * 180)
        self.map_fix = None
        self.moved = {"p": None, "t": time.time(), "sent": False}
        self.t0 = time.time()
        self.errors = 0
        self.settle_until = 0.0
        self.no_view = {}                        # (x, y) 경계 → 처음 "갈 자리 없음" 본 시각 (보류)
        self.idle_t = None                       # 갈 데가 보류 경계뿐인 상태가 시작된 시각
        self.last_status = time.time()

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, "map", lambda m: setattr(self, "grid", m), latched)
        self.create_subscription(OccupancyGrid, "global_costmap/costmap", lambda m: setattr(self, "cmap", m), latched)
        self.create_subscription(PointCloud2, "whisker/cloud", self.on_whisker_cloud, 1)
        self.create_subscription(String, "sense", self.on_sense, 10)
        self.create_subscription(Odometry, "odom", self.on_odom, 20)
        self.create_subscription(String, "whisker/state", self.on_wstate, 10)
        # 더듬기만 /cmd_vel 에 직접 냄 (Nav2 는 그동안 목표 없음). Whisker 가 반응하면 브리지가 /whisker/cmd_vel 을 우선 따름 → 노드끼리 안 깨짐
        self.cmd_pub = self.create_publisher(TwistStamped, "cmd_vel", 10)
        self.escape_pub = self.create_publisher(Empty, "whisker/escape", 1)   # 11-10 끼었음 → Whisker 탈출 모듈
        self.create_timer(0.1, self.probe_tick)
        for t in ("pose", "amcl_pose"):
            self.create_subscription(PoseWithCovarianceStamped, t, self.on_map_pose, 10)
        self.marker_pub = self.create_publisher(MarkerArray, "scr_explore/markers", 1)
        self.state_pub = self.create_publisher(String, "robot/state", 10)   # 11-10 상태 (화면·말하기가 같은 걸 봄)
        self.n_open = 0
        self.nav = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.planner = ActionClient(self, ComputePathToPose, "compute_path_to_pose")   # 더듬기: 길만 받음
        self.create_timer(1.0, self.step)
        self.say("시작 — Nav2 기다리는 중")

    def set_owner(self, who, why):
        if who != self.owner:
            self.say(f"바퀴 주인: {self.owner} → {who} ({why})")
            self.owner = who
            self.publish_state()

    def doing(self):
        """지금 하는 일 (짧은 영어 — OLED 는 한글이 안 될 수 있음)"""
        if self.phase in ("home", "done"):
            return self.phase.upper()
        if self.probe:
            return "PROBE" if self.probe.get("mode") == "probe" else "CRAWL"
        if self.goal:
            return "VIEW" if self.goal["stage"] == 1 else "EDGE"
        if self.paused:
            return "WAIT"
        if time.time() < self.settle_until:
            return "LOOK"
        return "THINK"

    def publish_state(self):
        """/robot/state: 목적(mode) · 바퀴 주인(owner) · 하는 일(doing) · 남은 경계 · 포기 수"""
        self.state_pub.publish(String(data=json.dumps({"mode": "explore", "phase": self.phase, "owner": self.owner,
                                                       "doing": self.doing(), "open": self.n_open, "gave_up": len(self.gave_up),
                                                       "t": round(time.time() - self.t0)})))

    def say(self, text, warn=False):
        if warn:                       # 같은 줄에서 info/warn 을 바꿔 쓰면 rclpy 가 ValueError (11-07 02:07)
            self.get_logger().warn("EXPLORE " + text)
        else:
            self.get_logger().info("EXPLORE " + text)

    # ------------------------------------------------------------------ 입력
    @safe
    def on_whisker_cloud(self, m):
        raw = bytes(m.data)
        self.whisker = [tuple(np.frombuffer(raw[i:i + 8], dtype="<f4")) for i in range(0, len(raw), m.point_step)]

    @safe
    def on_sense(self, m):
        try:
            d = json.loads(m.data)
        except ValueError:
            return
        if self.probe and d.get("ev") in CONTACTS and not d.get("observe"):
            self.probe["contacts"] += 1
            self.say(f"더듬다 Whisker 접촉 {self.probe['contacts']}/{WHISKER_GIVEUP} ({d.get('ev')})")
        tg = self.goal or self.paused                  # 쉬는 목표에도 접촉은 이어서 셈
        if tg and d.get("ev") in CONTACTS and not d.get("observe"):
            tg["contacts"] += 1
            self.say(f"가는 중 Whisker 접촉 {tg['contacts']}/{WHISKER_GIVEUP} ({d.get('ev')})")

    def on_odom(self, m):
        st, pp = m.header.stamp, m.pose.pose
        o = (st.sec + st.nanosec * 1e-9, pp.position.x, pp.position.y, yaw_of(pp.orientation))
        self.odom_hist.append(o)
        if self.moved["p"] is None or math.hypot(o[1] - self.moved["p"][0], o[2] - self.moved["p"][1]) > STUCK_M:
            self.moved.update(p=(o[1], o[2]), t=time.time(), sent=False)

    @safe
    def on_wstate(self, m):
        try:
            st = json.loads(m.data).get("state", "drive")
        except ValueError:
            return
        if st != self.wstate:
            self.wstate, self.wstate_t = st, time.time()
        if st in WHISKER_BUSY:
            self.set_owner("whisker", st)
        elif self.owner == "whisker":
            self.set_owner("none", f"Whisker 돌려줌 ({st})")
        if st in WHISKER_BUSY:                   # Whisker 가 맡음 → Nav2 는 쉬게 (서로의 시계가 겹치지 않게)
            if self.goal and not self.goal.get("cancel_t"):
                self.say(f"Whisker 가 맡음 ({st}) → Nav2 목표 잠시 내려놓음")
                self.cancel_goal("pause")
            elif self.phase == "home" and self.home_h and not self.home_paused:
                self.home_paused = True
                self.say(f"Whisker 가 맡음 ({st}) → 집 가기 잠시 멈춤")
                self.home_h.cancel_goal_async()

    def whisker_free(self):
        return self.wstate not in WHISKER_BUSY and time.time() - self.wstate_t >= 1.0

    def on_map_pose(self, m):
        st, pp = m.header.stamp, m.pose.pose
        self.map_fix = (st.sec + st.nanosec * 1e-9, pp.position.x, pp.position.y, yaw_of(pp.orientation))

    def robot(self):
        p = self.robot_pose()
        return p[:2] if p else None

    def robot_pose(self):
        """지도 위치·방향 = 마지막 SLAM 위치 + 그 뒤 odom 이동 (heartbeat·whisker 와 같은 방식)"""
        if not self.odom_hist:
            return None
        _, ox, oy, oyaw = self.odom_hist[-1]
        if not self.map_fix:
            # 11-06: SLAM 은 0.15 m 움직여야 첫 /pose 를 냄 → 출발 전엔 없음 → 탐색기가 영원히 "기다리는 중" 이었음.
            # SLAM 시작 땐 지도 = 출발 자리 기준이라 odom ≈ map → odom 위치를 그대로 씀 (첫 보정이 오면 그걸로)
            return ox, oy, oyaw
        ts, mx, my, myaw = self.map_fix
        old = min(self.odom_hist, key=lambda r: abs(r[0] - ts))
        dx, dy = ox - old[1], oy - old[2]
        c, s = math.cos(-old[3]), math.sin(-old[3])
        lx, ly = c * dx - s * dy, s * dx + c * dy
        c, s = math.cos(myaw), math.sin(myaw)
        return mx + c * lx - s * ly, my + s * lx + c * ly, wrap(myaw + oyaw - old[3])

    # ------------------------------------------------------------------ 경계 찾기
    def find_frontiers(self):
        """explore_lite 의 철학 (11-07 Jasper: "너무 멍청하게 구석에 가려 함"):
          경계 = SLAM 지도의 "아는 빈칸 ↔ 모르는 칸" (채워야 할 건 SLAM 지도. 11-07 02:29: 비용지도는 로봇 따라 움직이는 창이고
                 LiDAR 가 지나간 칸을 빈칸으로 지워서 경계가 안 보임 → 30 초 만에 "경계 없음")
          1. 로봇에서 SLAM 빈칸으로 퍼져 닿는 경계 전부 (원본과 같음. 11-08: 비용지도·35 cm 제한으로 지웠던 것 되돌림)
          2. 경계 위가 아니라 경계가 직선으로 보이는 안전한 칸(비용 < 40, 경계에서 가까운 곳) = 관찰점으로 감 (원본의 약점:
             목표가 구석·벽 옆에 찍혀 Nav2 가 버벅임 → 우리 방식). 그래도 그대로면 2단계로 경계에 직접"""
        g, cm = self.grid, self.cmap
        if g is None or cm is None:
            return []
        w, h, res = g.info.width, g.info.height, g.info.resolution
        ox, oy = g.info.origin.position.x, g.info.origin.position.y
        a = np.array(g.data, dtype=np.int16).reshape(h, w)            # SLAM 지도
        # 비용지도를 SLAM 지도 칸에 맞춰 옮김 (해상도·원점이 달라도: 칸 중심 좌표로 찾아봄). 비용지도 밖 = 모름(-1)
        cw, ch, cres = cm.info.width, cm.info.height, cm.info.resolution
        cox, coy = cm.info.origin.position.x, cm.info.origin.position.y
        craw = np.array(cm.data, dtype=np.int16).reshape(ch, cw)
        yy, xx = np.mgrid[:h, :w]
        ci = np.floor((oy + (yy + 0.5) * res - coy) / cres).astype(int)
        cj = np.floor((ox + (xx + 0.5) * res - cox) / cres).astype(int)
        inside = (ci >= 0) & (ci < ch) & (cj >= 0) & (cj < cw)
        cost = np.full((h, w), -1, np.int16)
        cost[inside] = craw[ci[inside], cj[inside]]
        rx, ry = self.robot()
        r0, c0 = int((ry - oy) / res), int((rx - ox) / res)
        passable = (cost >= 0) & (cost < REACH_COST)
        seed = passable & ((yy - r0) ** 2 + (xx - c0) ** 2 <= (0.3 / res) ** 2)   # 로봇이 벽 옆이면 칸 자체는 비쌀 수 있음
        def spread(seed, ok):                              # 갈 수 있는 칸으로 퍼지기 (4방향)
            reach = seed & ok
            while True:
                p = np.pad(reach, 1)
                nxt = (reach | p[:-2, 1:-1] | p[2:, 1:-1] | p[1:-1, :-2] | p[1:-1, 2:]) & ok
                if (nxt == reach).all():
                    return reach
                reach = nxt
        reach = spread(seed, passable)
        near_seed = (cost >= 0) & ((yy - r0) ** 2 + (xx - c0) ** 2 <= (0.3 / res) ** 2)
        approach = spread(near_seed, (cost >= 0) & (cost < APPROACH_COST))   # 2단계용: 벽에 더 붙어도 됨
        # 경계 = 로봇에서 SLAM 지도의 빈칸으로 퍼져서 닿는 "빈칸↔모르는 칸" 전부 (explore_lite 원본과 같음, 거리 제한 없음).
        #   11-08: 부풀린 비용지도 + "갈 수 있는 칸 35 cm 안" 제한(11-07 내가 붙인 것)이 11시 통로·오른쪽 벽 경계를 지워
        #   "다 봤다"로 끝냄 → 뺌. 갈 수 있나는 Nav2·Whisker 가 판단하고, 못 가면 이유와 함께 포기
        free = a == 0
        seed2 = free & ((yy - r0) ** 2 + (xx - c0) ** 2 <= (0.3 / res) ** 2)
        near = seed2.copy()
        while True:
            p = np.pad(near, 1)
            nxt = (near | p[:-2, 1:-1] | p[2:, 1:-1] | p[1:-1, :-2] | p[1:-1, 2:]) & free
            if (nxt == near).all():
                break
            near = nxt
        pad = np.pad(a == -1, 1)
        near_unk = pad[:-2, 1:-1] | pad[2:, 1:-1] | pad[1:-1, :-2] | pad[1:-1, 2:]
        fr = free & near_unk & near
        seen = np.zeros_like(fr)
        safe_r, safe_c = np.nonzero(reach & (cost < VIEW_COST))
        reach_r, reach_c = np.nonzero(approach)
        out = []
        for r0_, c0_ in zip(*np.nonzero(fr)):
            if seen[r0_, c0_]:
                continue
            cells, q = [], [(r0_, c0_)]
            seen[r0_, c0_] = True
            while q:                                      # 8방향으로 이어진 경계 칸 묶기
                r, c = q.pop()
                cells.append((r, c))
                for dr in (-1, 0, 1):
                    for dc in (-1, 0, 1):
                        r2, c2 = r + dr, c + dc
                        if 0 <= r2 < h and 0 <= c2 < w and fr[r2, c2] and not seen[r2, c2]:
                            seen[r2, c2] = True
                            q.append((r2, c2))
            size = len(cells) * res
            if size < MIN_FRONTIER:
                continue
            mr = sum(c[0] for c in cells) / len(cells)
            mc = sum(c[1] for c in cells) / len(cells)
            gr, gc = min(cells, key=lambda c: (c[0] - mr) ** 2 + (c[1] - mc) ** 2)   # 중심에 가장 가까운 경계 칸
            vp = self.viewpoint(a, safe_r, safe_c, gr, gc, r0, c0, res)
            f = {"x": ox + (gc + 0.5) * res, "y": oy + (gr + 0.5) * res, "size": size,
                 "cells": [(ox + (c + 0.5) * res, oy + (r + 0.5) * res) for r, c in cells], "vx": None, "vy": None,
                 "ax": None, "ay": None}
            if len(reach_r):                              # 2단계 목표: 경계에 가장 가까운, 비용 < 85 인 칸 (경계 칸 자체 X — 그건 MPPI 가 거부)
                i = int(np.argmin((reach_r - gr) ** 2 + (reach_c - gc) ** 2))
                f["ax"], f["ay"] = ox + (reach_c[i] + 0.5) * res, oy + (reach_r[i] + 0.5) * res
            if vp:
                f["vx"], f["vy"] = ox + (vp[1] + 0.5) * res, oy + (vp[0] + 0.5) * res
            elif len(safe_r):                             # 왜 없는지 (no_view 로그용): 갈 수 있는 안전한 칸이 경계에서 얼마나 먼가
                f["why"] = f"가장 가까운 안전 칸 {float(np.min(np.hypot(safe_r - gr, safe_c - gc))) * res:.2f} m"
            else:
                f["why"] = "갈 수 있는 안전 칸 0개"
            out.append(f)
        return out

    @staticmethod
    def viewpoint(a, safe_r, safe_c, gr, gc, r0, c0, res):
        """경계 칸(gr, gc)이 직선으로 보이는 안전한 칸 중 "경계에" 가까운 곳. 시선은 SLAM 지도에서 아는 칸·벽 아닌 칸만.
        11-07 02:29: 로봇에 가까운 곳을 고르니 0.13 m 만 가고 끝 → SLAM 은 0.15 m 움직여야 스캔을 넣음 → 지도 그대로 → unseen"""
        if len(safe_r) == 0:
            return None
        d_f = np.hypot(safe_r - gr, safe_c - gc) * res
        ok = (d_f >= VIEW_MIN) & (d_f <= VIEW_MAX)
        sr, sc, d_f = safe_r[ok], safe_c[ok], d_f[ok]
        if len(sr) == 0:
            return None
        score = d_f + 0.3 * np.hypot(sr - r0, sc - c0) * res
        for i in np.argsort(score)[:60]:
            n = int(max(abs(sr[i] - gr), abs(sc[i] - gc))) + 1
            lr = np.rint(np.linspace(sr[i], gr, n)).astype(int)[:-2]
            lc = np.rint(np.linspace(sc[i], gc, n)).astype(int)[:-2]
            v = a[lr, lc]                                  # SLAM 지도: 아는 칸이고 벽(점유)이 아님
            if ((v >= 0) & (v < 50)).all():
                return int(sr[i]), int(sc[i])
        i = int(np.argmin(d_f))                            # 시선이 다 막힘 → 경계에 가장 가까운 안전한 칸으로 다가감
        return int(sr[i]), int(sc[i])

    @staticmethod
    def near(x, y, pts, r=REGION):
        return any(math.hypot(x - p[0], y - p[1]) < r for p in pts)

    def give_up(self, x, y, reason, why=None):
        self.gave_up.append({"x": round(x, 3), "y": round(y, 3), "reason": reason})
        self.say(f"포기 ({reason}{', ' + why if why else ''}) @({x:.2f}, {y:.2f}) — 지금까지 포기 {len(self.gave_up)}곳", warn=True)

    # ------------------------------------------------------------------ 한 걸음 (1 s 마다)
    @safe
    def step(self):
        if self.phase == "done":
            return
        waiting = [w for w, ok in (("Nav2", self.nav.server_is_ready()), ("지도", self.grid is not None),
                                   ("비용지도", self.cmap is not None),
                                   ("odom", self.robot() is not None)) if not ok]
        if waiting:
            if int(time.time() - self.t0) % 5 == 0:
                self.say("기다리는 중: " + ", ".join(waiting))
            return
        rx, ry = self.robot()
        now = time.time()
        if not self.moved["sent"] and now - self.moved["t"] > STUCK_S and self.phase != "done":
            self.moved["sent"] = True
            self.say(f"{STUCK_S / 60:.0f}분 동안 제자리 — 멈춘 것 같음", warn=True)
            threading.Thread(target=ntfy, args=("robot: explore STUCK?", f"{STUCK_S / 60:.0f}분 동안 10 cm 도 못 움직임. RViz 확인", True),
                             daemon=True).start()

        self.frontiers = self.find_frontiers()
        given = [(g["x"], g["y"]) for g in self.gave_up]
        cands, held = [], []
        for f in self.frontiers:
            if self.near(f["x"], f["y"], given):
                continue
            if self.near(f["x"], f["y"], self.whisker):
                self.give_up(f["x"], f["y"], "whisker")       # 닿아서 확인한 곳 → 처음부터 제외
                given.append((f["x"], f["y"]))
                continue
            key = next((k for k in self.no_view if math.hypot(k[0] - f["x"], k[1] - f["y"]) < REGION), None)
            if f["vx"] is not None and key is not None:
                del self.no_view[key]                          # 관찰점이 다시 생김 → "없음" 기록 지움 (11-08 22:14: 초반 기록이
                key = None                                     #   남아 있다가 잠깐 사라진 순간 바로 "30 s 지남" → 11시 가는 중 포기)
            going = any(t and math.hypot(t["x"] - f["x"], t["y"] - f["y"]) < REGION for t in (self.goal, self.probe, self.paused))
            #   (11-08 22:54: 더듬는 중 10 cm 옆 경계가 no_view 로 따로 포기됨 → 더듬기·쉬는 목표도 보호)
            if f["vx"] is None and going:
                cands.append(f)                                # 지금 가고 있는 목표는 이 규칙으로 포기하지 않음
                continue
            if f["vx"] is None:                                # 근처에 안전한 자리가 없음 → 보류 (다른 데 다 보고도 30 s 그대로면 포기)
                if key is None:
                    self.no_view[(f["x"], f["y"])] = now
                held.append(f)
                continue
            cands.append(f)
        # no_view 포기는 "할 일이 없을 때"만 센다 (11-08 23:38 거실: 다른 곳을 보러 가는 동안 30 s 가 지나 9곳 한꺼번에 포기.
        #   돌아다니면 지도·비용지도가 바뀌어 관찰점이 생길 수 있음)
        busy = self.goal or self.probe or self.paused or now < self.settle_until or any(f["vx"] is not None for f in cands)
        if busy or not held:
            self.idle_t = None
        elif self.idle_t is None:
            self.idle_t = now
        elif now - self.idle_t > NOVIEW_S:
            for f in held:
                self.give_up(f["x"], f["y"], "no_view", f.get("why"))
                given.append((f["x"], f["y"]))
                k = next((k for k in self.no_view if math.hypot(k[0] - f["x"], k[1] - f["y"]) < REGION), None)
                if k:
                    del self.no_view[k]
            held, self.idle_t = [], None
        self.n_open = len(cands) + len(held)
        self.publish_markers()
        self.publish_state()

        if self.phase == "home":
            if self.home_paused and self.whisker_free():
                self.home_paused = False
                self.say("Whisker 가 돌려줌 → 집으로 다시")
                self.send(0.0, 0.0, rx, ry, home=True)
            return                                             # 결과 콜백이 마무리
        if now < self.settle_until:
            return                                             # 관찰점 도착 — 지도가 새로 그려질 때까지
        if self.probe:
            p = self.probe
            if not any(math.hypot(c[0] - p["x"], c[1] - p["y"]) < 0.3 for f in self.frontiers for c in f["cells"]):
                self.end_probe("seen")                         # 더듬다가 경계가 보임
            return                                             # 더듬기는 probe_tick 이 몰고 감
        g = self.goal
        if g and g.get("cancel_t") and now - g["cancel_t"] > CANCEL_WAIT:
            # 취소를 보냈는데 Nav2 가 답을 안 함 → 우리가 정리하고 다음으로 (멈춰 서 있지 않게)
            self.say(f"취소 답이 {CANCEL_WAIT:.0f}초 넘게 없음 → 우리가 정리", warn=True)
            if g["cancel_reason"] == "pause":
                self.paused = g
            elif g["cancel_reason"] == "near":
                self.viewed.append((g["x"], g["y"]))
                self.settle_until = now + SETTLE_S
            elif g["cancel_reason"] == "stall":
                self.approached.append((g["x"], g["y"]))
            elif g["cancel_reason"] == "takeover":
                self.goal = None
                self.start_takeover(g)
                return
            elif g["cancel_reason"] != "replan":
                self.give_up(g["x"], g["y"], g["cancel_reason"])
            self.goal = None
            return                                             # 다음 걸음에서 포기 목록을 반영해 새로 고름
        if now - self.last_status > STATUS_S:
            self.last_status = now
            self.say(f"상태: {self.phase}, 위치 ({rx:.2f}, {ry:.2f}), 경계 {len(cands)}곳, 포기 {len(self.gave_up)}곳" +
                     (f", 목표 ({g['x']:.2f}, {g['y']:.2f}) 관찰점까지 {math.hypot(rx - g['vx'], ry - g['vy']):.2f} m, "
                      f"진척 {now - g['best_t']:.0f}초 전, 접촉 {g['contacts']}" if g else ", 목표 없음"))
        if not self.whisker_free():
            return                                             # Whisker 가 몰고 있음 → 우리 판단·시계 모두 쉼
        if self.paused:
            pg, self.paused = self.paused, None
            if pg["contacts"] >= WHISKER_GIVEUP:
                self.give_up(pg["x"], pg["y"], "whisker")
            elif any(math.hypot(c[0] - pg["x"], c[1] - pg["y"]) < 0.3 for f in self.frontiers for c in f["cells"]):
                self.say("Whisker 가 돌려줌 → 같은 목표 다시 (Nav2 에 맡김)")
                self.send(pg["x"], pg["y"], rx, ry, view=(pg["vx"], pg["vy"]), stage=pg["stage"], contacts=pg["contacts"])
            else:
                self.say("Whisker 가 돌려줌 — 그 경계는 이미 보임 → 다음 목표")
            return
        if g:
            if g.get("cancel_t"):
                return                                         # 취소 답 기다리는 중
            dist = math.hypot(rx - g["vx"], ry - g["vy"])
            if dist < g["best"] - PROGRESS_M:
                g["best"], g["best_t"] = dist, now
            still = any(math.hypot(c[0] - g["x"], c[1] - g["y"]) < 0.3 for f in self.frontiers for c in f["cells"])
            if g["contacts"] >= WHISKER_GIVEUP:
                self.cancel_goal("whisker")
            elif g["stage"] == 2 and now - g["best_t"] > STALL_S and math.hypot(rx - g["x"], ry - g["y"]) < PROBE_NEAR:
                self.say(f"경계 옆에서 Nav2 가 {STALL_S:.0f}초 멈칫 → 더듬기로")
                self.cancel_goal("stall")
            elif g["stage"] == 1 and now - g["best_t"] > STALL_S and math.hypot(rx - g["vx"], ry - g["vy"]) < VIEW_NEAR:
                self.say(f"관찰점 {math.hypot(rx - g['vx'], ry - g['vy']):.2f} m 앞에서 Nav2 가 {STALL_S:.0f}초 멈칫 → 여기서 본 걸로")
                self.cancel_goal("near")                   # (11-08 23:38: 37 cm 앞에서 MPPI 0 명령 45 s → no_progress 포기)
            elif now - g["best_t"] > STALL_S and not g.get("crawled"):
                self.say(f"Nav2 가 {STALL_S:.0f}초 진척 없음 (길 남은 거리 {g.get('rem', -1):.2f} m) → crawl 이 이어받음")
                self.cancel_goal("takeover")
            elif now - g["best_t"] > NO_PROGRESS_S:
                self.cancel_goal("no_progress")
            elif not still:
                self.say("목표 경계가 이미 보임 → 다음 목표")
                self.cancel_goal(None)
            return
        if not cands and held:
            if int(now) % 5 == 0:
                self.say(f"갈 자리 없는 경계 {len(held)}곳 보류 중 (할 일 없이 {NOVIEW_S:.0f}초 지나면 포기)")
            return
        if not cands:
            self.say(f"남은 경계 없음 (포기 {len(self.gave_up)}곳) → 집으로")
            self.phase = "home"
            self.send(0.0, 0.0, rx, ry, home=True)
            return
        cands = [f for f in cands if f["vx"] is not None]       # (가는 중 목표 보호용으로 넣은 것은 고를 때 빼기)
        if not cands:
            return
        best = min(cands, key=lambda f: W_DIST * math.hypot(f["vx"] - rx, f["vy"] - ry) - W_SIZE * f["size"])
        if self.near(best["x"], best["y"], self.reached, 0.3):
            self.give_up(best["x"], best["y"], "unseen")     # 더듬기까지 했는데도 그대로 = LiDAR 로 못 보는 곳
            return
        if self.near(best["x"], best["y"], self.approached, 0.3):
            self.start_probe(best)                            # 3단계: 경계 옆까지 왔는데 그대로 → 더듬기
            return
        if self.near(best["x"], best["y"], self.viewed, 0.3):
            # 2단계: 관찰점에서 봤는데 그대로 → 경계에 가장 가까운 싼 칸으로. 그 칸이 지금 자리보다 10 cm 이상 가깝지 않으면
            #   Nav2 로 갈 의미가 없음 (이미 거기서 봤음) → 바로 더듬기 (Jasper 11-08: "들어가려다 위스커로 포기")
            ax, ay = best.get("ax"), best.get("ay")
            here = math.hypot(rx - best["x"], ry - best["y"])
            if ax is not None and math.hypot(ax - best["x"], ay - best["y"]) < here - 0.10:
                self.say(f"봤는데 그대로 → 경계 옆 싼 칸 ({ax:.2f}, {ay:.2f}) 으로")
                self.send(best["x"], best["y"], rx, ry, view=(ax, ay), stage=2)
            else:
                self.approached.append((best["x"], best["y"]))
                self.start_probe(best)
            return
        self.send(best["x"], best["y"], rx, ry, view=(best["vx"], best["vy"]), stage=1)

    # ------------------------------------------------------------------ Nav2
    def send(self, x, y, rx, ry, home=False, view=None, stage=1, contacts=0, crawled=False):
        """x, y = 경계 (포기·기억은 경계 기준), view = 실제로 갈 관찰점"""
        vx, vy = view if view else (x, y)
        ps = PoseStamped()
        ps.header.frame_id = "map"
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x, ps.pose.position.y = float(vx), float(vy)
        yaw = 0.0 if home else math.atan2(y - vy, x - vx)       # 경계 쪽을 보게 (탐색 모드는 도착 방향 안 봄)
        ps.pose.orientation.z, ps.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        goal = NavigateToPose.Goal()
        goal.pose = ps
        if not home:
            self.goal = {"x": x, "y": y, "vx": vx, "vy": vy, "stage": stage, "t0": time.time(), "best": math.hypot(vx - rx, vy - ry),
                         "best_t": time.time(), "contacts": contacts, "handle": None, "cancel_reason": None,
                         "rem": -1.0, "rem_best": None, "crawled": crawled}
            self.say(f"목표 경계 ({x:.2f}, {y:.2f}) ← {'관찰점' if stage == 1 else '직접'} ({vx:.2f}, {vy:.2f}) — 거리 {math.hypot(vx - rx, vy - ry):.2f} m")
        g = None if home else self.goal
        self.set_owner("nav2", "집으로" if home else f"{stage}단계 목표")
        fut = self.nav.send_goal_async(goal, feedback_callback=lambda fb: self.on_feedback(fb, g))
        fut.add_done_callback(lambda f: self.on_accepted(f, home, g))

    def on_feedback(self, fb, g):
        """Nav2 가 주행 중 알려주는 "길 따라 남은 거리" — 돌아가는 길에서도 줄어듦 (직선거리는 늘 수 있음)"""
        if g is None or self.goal is not g:
            return
        d = float(fb.feedback.distance_remaining)
        if d <= 0.0:
            return                                   # 아직 길이 없음
        g["rem"] = d
        if g["rem_best"] is None or d < g["rem_best"] - PROGRESS_M:
            g["rem_best"], g["best_t"] = d, time.time()

    @safe
    def on_accepted(self, fut, home, g):
        h = fut.result()
        if not h.accepted:
            if home:
                self.finish(False)
            elif g is not None and self.goal is g:
                self.give_up(g["x"], g["y"], "rejected")
                self.goal = None
            return
        if home:
            self.home_h = h
        if g is not None:
            g["handle"] = h
            if g.get("cancel_reason") and self.goal is g:   # 받아지기 전에 취소하기로 했던 것
                h.cancel_goal_async()
        h.get_result_async().add_done_callback(lambda f: self.on_result(f, home, g))

    def cancel_goal(self, reason):
        g = self.goal
        if not g or g.get("cancel_t"):
            return                                   # 이미 취소 보냄 (매초 다시 보내지 않음)
        g["cancel_reason"], g["cancel_t"] = reason or "replan", time.time()
        self.say(f"목표 취소 ({g['cancel_reason']}) — Nav2 답 기다림")
        if g["handle"]:
            g["handle"].cancel_goal_async()

    @safe
    def on_result(self, fut, home, g=None):
        status = fut.result().status            # 4 SUCCEEDED, 5 CANCELED, 6 ABORTED
        if home:
            if self.home_paused:
                return                           # Whisker 때문에 잠시 내려놓은 집 가기 → step 이 다시 보냄
            self.finish(status == 4)
            return
        if g is None or self.goal is not g:
            return                               # 이미 정리한 옛 목표의 늦은 답
        self.goal = None
        self.say(f"Nav2 답: {dict(((4, '도착'), (5, '취소됨'), (6, '실패'))).get(status, status)}")
        if self.owner == "nav2":
            self.set_owner("none", "Nav2 답")
        if g.get("cancel_reason") == "takeover":
            self.start_takeover(g)                              # 포기 아님 → crawl 이 같은 목표로
            return
        if g.get("cancel_reason") == "pause":
            self.paused = g                                     # 포기 아님 → Whisker 가 돌려주면 다시 보냄
            return
        if g.get("cancel_reason") == "near":
            self.viewed.append((g["x"], g["y"]))                # 관찰점 바로 앞 = 거기서 본 것과 같음 → 지도 기다렸다 다음 단계
            self.settle_until = time.time() + SETTLE_S
            self.say(f"관찰점 앞에서 봄 (경계 {g['x']:.2f}, {g['y']:.2f}) — 지도 갱신 {SETTLE_S:.0f}초 기다림")
        elif g.get("cancel_reason") == "stall":
            self.approached.append((g["x"], g["y"]))            # 포기 아님 → 다음 걸음에서 더듬기
        elif status == 4:
            (self.approached if g.get("stage") == 2 else self.viewed).append((g["x"], g["y"]))
            self.settle_until = time.time() + SETTLE_S
            where = "경계 옆" if g.get("stage") == 2 else "관찰점"
            self.say(f"{where} 도착 (경계 {g['x']:.2f}, {g['y']:.2f}) — 지도 갱신 {SETTLE_S:.0f}초 기다림")
        elif g.get("cancel_reason") and g["cancel_reason"] != "replan":
            self.give_up(g["x"], g["y"], g["cancel_reason"])
        elif status == 6:
            if not self.wedged(g["x"], g["y"], "Nav2 실패"):
                self.give_up(g["x"], g["y"], "no_path")

    # ------------------------------------------------------------------ 끼임 → 탈출 모듈
    def wedged(self, fx, fy, why, now_sure=False):
        """길 없음이 경계 탓인가, 로봇이 끼어서인가. 같은 자리(15 cm)에서 다른 경계로도 길 없음이 났거나(now_sure=False),
        crawl 의 플래너가 방금까지 가던 곳으로도 길을 못 만들면(now_sure=True) = 끼었음 → 탈출 요청, 포기 안 함.
        같은 자리 탈출이 WEDGE_ESC_MAX 번 넘으면 진짜 길 없음으로 봄 (False)"""
        pose = self.robot()
        if not pose:
            return False
        rx, ry = pose
        now = time.time()
        self.fails = [f for f in self.fails if now - f[0] < WEDGE_S]
        here = [f for f in self.fails if math.hypot(f[1] - rx, f[2] - ry) < WEDGE_R]
        self.fails.append((now, rx, ry, fx, fy))
        if not now_sure and not any(math.hypot(f[3] - fx, f[4] - fy) >= REGION for f in here):
            return False                                   # 이 자리 첫 실패 → 경계 탓일 수 있음
        self.escapes = [e for e in self.escapes if now - e[0] < WEDGE_S]
        if sum(1 for e in self.escapes if math.hypot(e[1] - rx, e[2] - ry) < WEDGE_R) >= WEDGE_ESC_MAX:
            self.say(f"끼임 탈출을 이 자리에서 {WEDGE_ESC_MAX}번 했는데도 길 없음 → 진짜 길 없음으로 봄", warn=True)
            return False
        for f in here:                                     # 이 자리에서 "길 없음"으로 포기했던 경계 되돌림 (로봇 탓이었음)
            back = [gv for gv in self.gave_up if gv["reason"] == "no_path" and math.hypot(gv["x"] - f[3], gv["y"] - f[4]) < 0.05]
            for gv in back:
                self.gave_up.remove(gv)
                self.say(f"포기 되돌림 ({gv['x']:.2f}, {gv['y']:.2f}) — 로봇이 끼어서 길이 없었던 것")
        self.escapes.append((now, rx, ry))
        self.say(f"끼임 ({why}) @({rx:.2f}, {ry:.2f}) → 경계 포기 대신 Whisker 탈출 요청", warn=True)
        self.set_owner("whisker", "끼임 탈출 요청")
        self.escape_pub.publish(Empty())
        return True

    def cost_at(self, x, y):
        """전역 비용지도 값 (0~100, 모름·밖 = -1)"""
        cm = self.cmap
        if cm is None:
            return -1
        i = int((y - cm.info.origin.position.y) / cm.info.resolution)
        j = int((x - cm.info.origin.position.x) / cm.info.resolution)
        if not (0 <= i < cm.info.height and 0 <= j < cm.info.width):
            return -1
        return cm.data[i * cm.info.width + j]

    def start_takeover(self, g):
        """Nav2 가 얼었음 → 같은 목표(관찰점·경계 옆 칸)로 플래너 길을 받아 우리가 기어감 (11-10)"""
        now = time.time()
        self.probe = {"x": g["x"], "y": g["y"], "tx": g["vx"], "ty": g["vy"], "mode": "takeover", "stage": g["stage"],
                      "t0": now, "contacts": g["contacts"], "going": False, "quiet_until": now + PROBE_QUIET,
                      "hold": 0.0, "path": None, "planning": False, "gen": 0, "i": 0, "limit": PROBE_S}
        self.set_owner("crawl", f"Nav2 얼음 → {g['stage']}단계 목표 ({g['vx']:.2f}, {g['vy']:.2f}) 로 기어감")

    # ------------------------------------------------------------------ 더듬기 (3단계)
    def start_probe(self, f):
        now = time.time()
        self.probe = {"x": f["x"], "y": f["y"], "t0": now, "contacts": 0, "going": False, "quiet_until": now + PROBE_QUIET,
                      "hold": 0.0, "path": None, "planning": False, "gen": 0, "i": 0, "limit": PROBE_S, "mode": "probe"}
        self.set_owner("crawl", "더듬기")
        self.say(f"더듬기 시작: 경계 ({f['x']:.2f}, {f['y']:.2f}) — 플래너 길을 천천히 따라감 (Whisker 켜짐)")

    def request_path(self):
        p = self.probe
        p["planning"], p["path"] = True, None
        ps = PoseStamped()
        ps.header.frame_id = "map"
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x, ps.pose.position.y = float(p.get("tx", p["x"])), float(p.get("ty", p["y"]))
        ps.pose.orientation.w = 1.0
        g = ComputePathToPose.Goal()
        g.goal, g.use_start, g.planner_id = ps, False, ""     # 출발 = 지금 로봇 자리. 경계 칸이 비싸도 NavFn 이 0.5 m 안 가까운 칸으로
        gen = p["gen"]
        self.planner.send_goal_async(g).add_done_callback(lambda f: self.on_plan_accepted(f, p, gen))

    @safe
    def on_plan_accepted(self, fut, p, gen):
        h = fut.result()
        if not h.accepted:
            if self.probe is p:
                self.end_probe("no_path")
            return
        h.get_result_async().add_done_callback(lambda f: self.on_plan(f, p, gen))

    @safe
    def on_plan(self, fut, p, gen):
        if self.probe is not p or p["gen"] != gen:
            return                                       # 옛 길 (그 사이 Whisker 가 빼서 다시 받기로 함)
        p["planning"] = False
        r = fut.result()
        poses = r.result.path.poses if r.status == 4 else []
        pts = [(q.pose.position.x, q.pose.position.y) for q in poses]
        if len(pts) < 2:
            return self.end_probe("no_path")
        length = sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(pts, pts[1:]))
        if length > PROBE_PATH_MAX:
            self.say(f"더듬기 길 {length:.2f} m > {PROBE_PATH_MAX} m")
            return self.end_probe("far")
        mid = [q for q in pts if math.hypot(q[0] - pts[0][0], q[1] - pts[0][1]) > 0.10
               and math.hypot(q[0] - pts[-1][0], q[1] - pts[-1][1]) > 0.15]   # 출발(지금 자리)·끝(경계 앞)은 빼고
        worst = max((self.cost_at(*q) for q in mid), default=-1)
        if worst >= ROTATE_COST:
            self.say(f"crawl 길이 돌 수 없는 좁은 곳을 지남 (비용 {worst} ≥ {ROTATE_COST}) → 안 들어감")
            return self.end_probe("narrow")
        p["path"], p["i"] = pts, 0
        p["limit"] = max(p["limit"], time.time() - p["t0"] + PROBE_S + length / 0.05)
        self.say(f"더듬기 길 받음: {length:.2f} m, 점 {len(pts)}개")

    def drive(self, x, z):
        m = TwistStamped()
        m.header.frame_id = "base_link"
        m.header.stamp = self.get_clock().now().to_msg()
        m.twist.linear.x, m.twist.angular.z = float(x), float(z)
        self.cmd_pub.publish(m)

    @safe
    def probe_tick(self):
        p = self.probe
        if not p:
            return
        now = time.time()
        if now < p["quiet_until"]:
            return                                       # Nav2 가 완전히 조용해질 때까지
        pose = self.robot_pose()
        if not pose:
            return
        rx, ry, yaw = pose
        if p["contacts"] >= WHISKER_GIVEUP:
            return self.end_probe("whisker")
        if self.wstate != "drive":
            p["hold"] = now                              # Whisker 가 빼는 중 (브리지는 어차피 Whisker 를 따름) → 기다림
            if p["path"] is not None or p["planning"]:
                p["path"], p["planning"], p["gen"] = None, False, p["gen"] + 1   # 빼고 나면 자리가 바뀜 → 길 다시 받기
            p["t0"] += now - p.get("last", now)         # 그동안 더듬기 시계도 멈춤 (빼기·갇힘 대기가 25 s 를 먹지 않게)
            p["last"] = now
            return
        p["last"] = now
        if now - p["hold"] < 1.0:
            return                                       # Whisker 가 막 돌려줌 → 1 s 숨 고르기
        if now - p["t0"] > p["limit"]:
            return self.end_probe("시간 끝")
        if p["path"] is None:
            if not p["planning"]:
                self.request_path()                      # 처음, 또는 Whisker 가 뺀 뒤 → 지금 자리에서 길을 새로 받음
            return
        pts = p["path"]
        i = p["i"]
        while i + 1 < len(pts) and math.hypot(pts[i + 1][0] - rx, pts[i + 1][1] - ry) <= math.hypot(pts[i][0] - rx, pts[i][1] - ry):
            i += 1                                       # 길 위에서 로봇과 가장 가까운 점 (앞으로만)
        p["i"] = i
        if math.hypot(pts[-1][0] - rx, pts[-1][1] - ry) < 0.06:
            return self.end_probe("다 감")
        j = i
        while j + 1 < len(pts) and math.hypot(pts[j][0] - rx, pts[j][1] - ry) < PROBE_LOOK:
            j += 1
        err = wrap(math.atan2(pts[j][1] - ry, pts[j][0] - rx) - yaw)
        if abs(err) > 0.6 or (not p["going"] and abs(err) > 0.3):   # 많이 틀어짐 → 제자리에서 돎 (0.3 rad/s 아래는 바닥선이 세게 돎)
            p["going"] = False
            self.drive(0.0, math.copysign(max(0.3, min(0.5, 1.2 * abs(err))), err))
            return
        p["going"] = True                                # 길 따라가기 (pure pursuit: 곡률 = 2 sin(오차) / 앞보기 거리)
        self.drive(PROBE_V, max(-0.4, min(0.4, 2 * PROBE_V * math.sin(err) / PROBE_LOOK)))

    def end_probe(self, why):
        p, self.probe = self.probe, None
        self.drive(0.0, 0.0)
        if self.owner == "crawl":
            self.set_owner("none", f"crawl 끝 ({why})")
        if p.get("mode") == "takeover" and why not in ("whisker", "seen"):
            if why == "다 감":                                # Nav2 대신 도착 = Nav2 도착과 같게
                (self.approached if p["stage"] == 2 else self.viewed).append((p["x"], p["y"]))
                self.settle_until = time.time() + SETTLE_S
                self.say(f"crawl 로 {'경계 옆' if p['stage'] == 2 else '관찰점'} 도착 (경계 {p['x']:.2f}, {p['y']:.2f}) — 지도 갱신 {SETTLE_S:.0f}초 기다림")
            elif why == "no_path":
                if not self.wedged(p["x"], p["y"], "crawl 플래너 길 없음", now_sure=True):
                    self.give_up(p["x"], p["y"], "no_path")
            elif why in ("far", "narrow"):                   # 기어갈 길이 아님 → Nav2 에게 다시 (이번엔 이어받기 없이 45 s 규칙)
                rp = self.robot()
                if rp:
                    self.send(p["x"], p["y"], rp[0], rp[1], view=(p["tx"], p["ty"]), stage=p["stage"], contacts=p["contacts"], crawled=True)
            else:
                self.give_up(p["x"], p["y"], "no_progress")
            return
        if why == "whisker":
            self.give_up(p["x"], p["y"], "whisker")       # 더듬다 2번 닿음 = 몸으로 확인한 못 가는 곳
            return
        if why == "seen":
            self.say("더듬다가 경계가 보임 → 다음 목표")
            return
        if why == "no_path" and self.wedged(p["x"], p["y"], "더듬기 플래너 길 없음"):
            return
        if why in ("no_path", "far", "narrow"):
            self.give_up(p["x"], p["y"], why)             # 플래너도 길이 없음 = 벽 너머 / 너무 돌아감 / 돌 수 없는 틈
            return
        self.reached.append((p["x"], p["y"]))
        self.settle_until = time.time() + SETTLE_S
        self.say(f"더듬기 끝 ({why}) — 지도 갱신 {SETTLE_S:.0f}초 기다림")

    # ------------------------------------------------------------------ 끝
    @safe
    def finish(self, home_ok):
        if self.phase == "done":
            return
        self.phase = "done"
        self.publish_state()
        path = Path.home() / "maps" / f"{self.name}_{time.strftime('%Y%m%d_%H%M')}"
        path.parent.mkdir(exist_ok=True)
        r = subprocess.run(["ros2", "run", "nav2_map_server", "map_saver_cli", "-f", str(path),
                            "--ros-args", "-p", "save_map_timeout:=10.0"], capture_output=True, text=True)
        ok = "Map saved successfully" in (r.stdout + r.stderr)
        reasons = collections.Counter(g["reason"] for g in self.gave_up)
        Path(str(path) + "_frontiers.json").write_text(json.dumps({
            "gave_up": self.gave_up, "open": [{"x": f["x"], "y": f["y"], "size": round(f["size"], 2)} for f in self.frontiers],
            "whisker": [[round(float(x), 3), round(float(y), 3)] for x, y in self.whisker], "home_ok": home_ok}, ensure_ascii=False))
        mins, secs = divmod(int(time.time() - self.t0), 60)
        why = ", ".join(f"{k} {v}" for k, v in reasons.items()) or "없음"
        body = f"{mins}분 {secs}초 · 포기 {len(self.gave_up)}곳 ({why}) · Whisker 표시 {len(self.whisker)}점\n지도: {path.name}"
        title = "robot: explore done" + ("" if home_ok else ", NOT home") + ("" if ok else " (지도 저장 실패)")
        self.say(f"끝 — {body.replace(chr(10), ' / ')}", warn=not (home_ok and ok))
        threading.Thread(target=ntfy, args=(title, body, not (home_ok and ok)), daemon=True).start()

    def publish_markers(self):
        ma = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        ma.markers.append(clear)
        stamp = self.get_clock().now().to_msg()

        def mk(mid, typ, color, scale):
            m = Marker()
            m.header.frame_id, m.header.stamp = "map", stamp
            m.ns, m.id, m.type, m.action = "scr_explore", mid, typ, Marker.ADD
            m.color = ColorRGBA(r=color[0], g=color[1], b=color[2], a=color[3])
            m.scale.x = m.scale.y = m.scale.z = scale
            m.pose.orientation.w = 1.0
            return m
        fr = mk(0, Marker.POINTS, (0.2, 0.4, 1.0, 0.8), 0.05)
        fr.points = [Point(x=x, y=y, z=0.02) for f in self.frontiers for x, y in f["cells"]]
        ma.markers.append(fr)
        for i, g in enumerate(self.gave_up):
            s = mk(10 + 2 * i, Marker.SPHERE, (1.0, 0.1, 0.1, 0.8), 0.12)
            s.pose.position = Point(x=g["x"], y=g["y"], z=0.05)
            t = mk(11 + 2 * i, Marker.TEXT_VIEW_FACING, (1.0, 0.3, 0.3, 1.0), 0.08)
            t.pose.position = Point(x=g["x"], y=g["y"], z=0.2)
            t.text = g["reason"]
            ma.markers += [s, t]
        if self.probe:                                  # 노란 공 = 더듬어 보는 경계
            s = mk(3, Marker.SPHERE, (1.0, 0.9, 0.1, 0.9), 0.12)
            s.pose.position = Point(x=self.probe["x"], y=self.probe["y"], z=0.05)
            ma.markers.append(s)
        if self.goal:                                   # 초록 공 = 갈 관찰점, 초록 작은 공 = 보려는 경계
            s = mk(1, Marker.SPHERE, (0.1, 1.0, 0.2, 0.9), 0.15)
            s.pose.position = Point(x=self.goal["vx"], y=self.goal["vy"], z=0.05)
            t = mk(2, Marker.SPHERE, (0.1, 1.0, 0.2, 0.5), 0.08)
            t.pose.position = Point(x=self.goal["x"], y=self.goal["y"], z=0.05)
            ma.markers += [s, t]
        self.marker_pub.publish(ma)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--name", default="explore", help="지도 이름 앞부분")
    a = ap.parse_args()
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)   # Ctrl+C 를 우리가 잡아야 목표 취소를 보낼 수 있음
    n = Explorer(a.name)
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        if n.probe:
            n.drive(0.0, 0.0)
        if n.goal and n.goal.get("handle"):
            n.goal["handle"].cancel_goal_async()
            rclpy.spin_once(n, timeout_sec=0.5)
        n.say("Ctrl+C — 목표 취소, 멈춤", warn=True)
    finally:
        n.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
