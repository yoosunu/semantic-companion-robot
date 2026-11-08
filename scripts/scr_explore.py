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
     가는 중 그 경계가 지도에서 사라지면 (이미 보임) → 목표 바꿈
  5. 경계 없음 → 집(0,0)으로 → 지도 + 경계(포기 이유) 자동 저장 + 폰 알림. 3분 동안 10 cm 도 못 움직이면 STUCK 알림
  RViz (/scr_explore/markers): 파랑 = 남은 경계, 빨강 공 + 글자 = 포기(이유), 초록 큰 공 = 갈 관찰점, 작은 공 = 보려는 경계

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
from geometry_msgs.msg import Point, PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import ColorRGBA, String
from visualization_msgs.msg import Marker, MarkerArray

MIN_FRONTIER = 0.30      # m, 이보다 작은 경계 묶음 무시
REGION = 0.40            # m, 같은 곳 / Whisker 표시 근처
W_DIST, W_SIZE = 3.0, 1.0
WHISKER_GIVEUP = 2       # 가는 중 접촉 이만큼이면 포기
NO_PROGRESS_S, PROGRESS_M = 45.0, 0.05   # 11-07: 90 s 는 카펫 표시 너머 목표에 98 s 버팀
STUCK_S, STUCK_M = 180.0, 0.10
REACH_COST = 75          # 전역 비용지도(0~100) 이 미만 = 로봇 중심이 지나갈 수 있는 칸 (몸 외접원 밖 ≈ 78)
VIEW_COST = 40           # 관찰점은 더 넉넉히 (벽·가구에서 떨어진 곳)
SEE_R = 0.35             # m, 갈 수 있는 곳에서 이 안의 경계만 = 그 너머는 좁은 틈·구석
VIEW_MIN, VIEW_MAX = 0.20, 1.00   # m, 관찰점 ↔ 경계 거리
SETTLE_S = 7.0           # s, 관찰점 도착 후 SLAM 지도 갱신(5 s) 기다림 (11-07 02:25: 도착 즉시 unseen 으로 전부 포기)
NOVIEW_S = 30.0          # s, 갈 자리가 이만큼 계속 없을 때만 포기 (처음엔 지도가 작아서 잠깐 없을 수 있음)
CONTACTS = ("bump", "lift", "slope", "stuck", "turn_blocked")   # Whisker v9 의 감각 (ghost·touch 는 접촉 아님)
CANCEL_WAIT = 15.0       # s, 취소 보냈는데 Nav2 답이 없으면 우리가 정리
STATUS_S = 30.0          # s, 상태 한 줄 주기


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
        self.reached = []                        # 2단계: 경계 칸까지 직접 간 경계 (x, y)
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
        self.last_status = time.time()

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, "map", lambda m: setattr(self, "grid", m), latched)
        self.create_subscription(OccupancyGrid, "global_costmap/costmap", lambda m: setattr(self, "cmap", m), latched)
        self.create_subscription(PointCloud2, "whisker/cloud", self.on_whisker_cloud, 1)
        self.create_subscription(String, "sense", self.on_sense, 10)
        self.create_subscription(Odometry, "odom", self.on_odom, 20)
        for t in ("pose", "amcl_pose"):
            self.create_subscription(PoseWithCovarianceStamped, t, self.on_map_pose, 10)
        self.marker_pub = self.create_publisher(MarkerArray, "scr_explore/markers", 1)
        self.nav = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.create_timer(1.0, self.step)
        self.say("시작 — Nav2 기다리는 중")

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
        if self.goal and d.get("ev") in CONTACTS and not d.get("observe"):
            self.goal["contacts"] += 1
            self.say(f"가는 중 Whisker 접촉 {self.goal['contacts']}/{WHISKER_GIVEUP} ({d.get('ev')})")

    def on_odom(self, m):
        st, pp = m.header.stamp, m.pose.pose
        o = (st.sec + st.nanosec * 1e-9, pp.position.x, pp.position.y, yaw_of(pp.orientation))
        self.odom_hist.append(o)
        if self.moved["p"] is None or math.hypot(o[1] - self.moved["p"][0], o[2] - self.moved["p"][1]) > STUCK_M:
            self.moved.update(p=(o[1], o[2]), t=time.time(), sent=False)

    def on_map_pose(self, m):
        st, pp = m.header.stamp, m.pose.pose
        self.map_fix = (st.sec + st.nanosec * 1e-9, pp.position.x, pp.position.y, yaw_of(pp.orientation))

    def robot(self):
        """지도 위치 = 마지막 SLAM 위치 + 그 뒤 odom 이동 (heartbeat·whisker 와 같은 방식)"""
        if not self.odom_hist:
            return None
        _, ox, oy, _ = self.odom_hist[-1]
        if not self.map_fix:
            # 11-06: SLAM 은 0.15 m 움직여야 첫 /pose 를 냄 → 출발 전엔 없음 → 탐색기가 영원히 "기다리는 중" 이었음.
            # SLAM 시작 땐 지도 = 출발 자리 기준이라 odom ≈ map → odom 위치를 그대로 씀 (첫 보정이 오면 그걸로)
            return ox, oy
        ts, mx, my, myaw = self.map_fix
        old = min(self.odom_hist, key=lambda r: abs(r[0] - ts))
        dx, dy = ox - old[1], oy - old[2]
        c, s = math.cos(-old[3]), math.sin(-old[3])
        lx, ly = c * dx - s * dy, s * dx + c * dy
        c, s = math.cos(myaw), math.sin(myaw)
        return mx + c * lx - s * ly, my + s * lx + c * ly

    # ------------------------------------------------------------------ 경계 찾기
    def find_frontiers(self):
        """explore_lite 의 철학 (11-07 Jasper: "너무 멍청하게 구석에 가려 함"):
          경계 = SLAM 지도의 "아는 빈칸 ↔ 모르는 칸" (채워야 할 건 SLAM 지도. 11-07 02:29: 비용지도는 로봇 따라 움직이는 창이고
                 LiDAR 가 지나간 칸을 빈칸으로 지워서 경계가 안 보임 → 30 초 만에 "경계 없음")
          1. 그중 로봇에서 "갈 수 있는 칸"(전역 비용지도 < 75)으로 퍼져 닿는 곳 근처만 — 좁은 틈·구석 너머는 없음
          2. 경계 위가 아니라 경계가 직선으로 보이는 안전한 칸(비용 < 40, 경계에서 가까운 곳) = 관찰점으로 감"""
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
        reach = seed.copy()
        while True:                                        # 갈 수 있는 칸으로 퍼지기 (4방향)
            p = np.pad(reach, 1)
            nxt = (reach | p[:-2, 1:-1] | p[2:, 1:-1] | p[1:-1, :-2] | p[1:-1, 2:]) & passable
            if (nxt == reach).all():
                break
            reach = nxt
        near = reach.copy()
        for _ in range(int(SEE_R / res)):
            p = np.pad(near, 1)
            near = near | p[:-2, 1:-1] | p[2:, 1:-1] | p[1:-1, :-2] | p[1:-1, 2:]
        pad = np.pad(a == -1, 1)
        near_unk = pad[:-2, 1:-1] | pad[2:, 1:-1] | pad[1:-1, :-2] | pad[1:-1, 2:]
        fr = (a == 0) & near_unk & near
        seen = np.zeros_like(fr)
        safe_r, safe_c = np.nonzero(reach & (cost < VIEW_COST))
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
                 "cells": [(ox + (c + 0.5) * res, oy + (r + 0.5) * res) for r, c in cells], "vx": None, "vy": None}
            if vp:
                f["vx"], f["vy"] = ox + (vp[1] + 0.5) * res, oy + (vp[0] + 0.5) * res
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

    def give_up(self, x, y, reason):
        self.gave_up.append({"x": round(x, 3), "y": round(y, 3), "reason": reason})
        self.say(f"포기 ({reason}) @({x:.2f}, {y:.2f}) — 지금까지 포기 {len(self.gave_up)}곳", warn=True)

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
            if f["vx"] is None:                                # 근처에 안전한 자리가 없음 → 보류, 30 s 계속이면 포기
                key = next((k for k in self.no_view if math.hypot(k[0] - f["x"], k[1] - f["y"]) < REGION), None)
                if key is None:
                    self.no_view[(f["x"], f["y"])] = now
                elif now - self.no_view[key] > NOVIEW_S:
                    self.give_up(f["x"], f["y"], "no_view")
                    given.append((f["x"], f["y"]))
                    del self.no_view[key]
                held.append(f)
                continue
            cands.append(f)
        self.publish_markers()

        if self.phase == "home":
            return                                             # 결과 콜백이 마무리
        if now < self.settle_until:
            return                                             # 관찰점 도착 — 지도가 새로 그려질 때까지
        g = self.goal
        if g and g.get("cancel_t") and now - g["cancel_t"] > CANCEL_WAIT:
            # 취소를 보냈는데 Nav2 가 답을 안 함 → 우리가 정리하고 다음으로 (멈춰 서 있지 않게)
            self.say(f"취소 답이 {CANCEL_WAIT:.0f}초 넘게 없음 → 우리가 정리", warn=True)
            if g["cancel_reason"] != "replan":
                self.give_up(g["x"], g["y"], g["cancel_reason"])
            self.goal = None
            return                                             # 다음 걸음에서 포기 목록을 반영해 새로 고름
        if now - self.last_status > STATUS_S:
            self.last_status = now
            self.say(f"상태: {self.phase}, 위치 ({rx:.2f}, {ry:.2f}), 경계 {len(cands)}곳, 포기 {len(self.gave_up)}곳" +
                     (f", 목표 ({g['x']:.2f}, {g['y']:.2f}) 관찰점까지 {math.hypot(rx - g['vx'], ry - g['vy']):.2f} m, "
                      f"진척 {now - g['best_t']:.0f}초 전, 접촉 {g['contacts']}" if g else ", 목표 없음"))
        if g:
            if g.get("cancel_t"):
                return                                         # 취소 답 기다리는 중
            dist = math.hypot(rx - g["vx"], ry - g["vy"])
            if dist < g["best"] - PROGRESS_M:
                g["best"], g["best_t"] = dist, now
            still = any(math.hypot(c[0] - g["x"], c[1] - g["y"]) < 0.3 for f in self.frontiers for c in f["cells"])
            if g["contacts"] >= WHISKER_GIVEUP:
                self.cancel_goal("whisker")
            elif now - g["best_t"] > NO_PROGRESS_S:
                self.cancel_goal("no_progress")
            elif not still:
                self.say("목표 경계가 이미 보임 → 다음 목표")
                self.cancel_goal(None)
            return
        if not cands and held:
            if int(now) % 5 == 0:
                self.say(f"갈 자리 없는 경계 {len(held)}곳 보류 중 ({NOVIEW_S:.0f}초 지나면 포기)")
            return
        if not cands:
            self.say(f"남은 경계 없음 (포기 {len(self.gave_up)}곳) → 집으로")
            self.phase = "home"
            self.send(0.0, 0.0, rx, ry, home=True)
            return
        best = min(cands, key=lambda f: W_DIST * math.hypot(f["vx"] - rx, f["vy"] - ry) - W_SIZE * f["size"])
        if self.near(best["x"], best["y"], self.reached, 0.3):
            self.give_up(best["x"], best["y"], "unseen")     # 경계까지 직접 갔는데도 그대로 = LiDAR 로 못 보는 곳
            return
        if self.near(best["x"], best["y"], self.viewed, 0.3):
            # 2단계: 관찰점에서 봤는데 그대로 → 경계 칸으로 직접 가 봄. 가다 Whisker 2번 → whisker / 길 없음 → no_path /
            #   45 s 못 다가감 → no_progress / 가서도 그대로 → unseen (Jasper 11-08: "들어가려다 위스커로 포기")
            self.say(f"봤는데 그대로 → 경계로 직접 가 봄 ({best['x']:.2f}, {best['y']:.2f})")
            self.send(best["x"], best["y"], rx, ry, view=None, stage=2)
            return
        self.send(best["x"], best["y"], rx, ry, view=(best["vx"], best["vy"]), stage=1)

    # ------------------------------------------------------------------ Nav2
    def send(self, x, y, rx, ry, home=False, view=None, stage=1):
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
                         "best_t": time.time(), "contacts": 0, "handle": None, "cancel_reason": None}
            self.say(f"목표 경계 ({x:.2f}, {y:.2f}) ← {'관찰점' if stage == 1 else '직접'} ({vx:.2f}, {vy:.2f}) — 거리 {math.hypot(vx - rx, vy - ry):.2f} m")
        g = None if home else self.goal
        fut = self.nav.send_goal_async(goal)
        fut.add_done_callback(lambda f: self.on_accepted(f, home, g))

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
            self.finish(status == 4)
            return
        if g is None or self.goal is not g:
            return                               # 이미 정리한 옛 목표의 늦은 답
        self.goal = None
        self.say(f"Nav2 답: {dict(((4, '도착'), (5, '취소됨'), (6, '실패'))).get(status, status)}")
        if status == 4:
            (self.reached if g.get("stage") == 2 else self.viewed).append((g["x"], g["y"]))
            self.settle_until = time.time() + SETTLE_S
            where = "경계까지 직접" if g.get("stage") == 2 else "관찰점"
            self.say(f"{where} 도착 (경계 {g['x']:.2f}, {g['y']:.2f}) — 지도 갱신 {SETTLE_S:.0f}초 기다림")
        elif g.get("cancel_reason") and g["cancel_reason"] != "replan":
            self.give_up(g["x"], g["y"], g["cancel_reason"])
        elif status == 6:
            self.give_up(g["x"], g["y"], "no_path")

    # ------------------------------------------------------------------ 끝
    @safe
    def finish(self, home_ok):
        if self.phase == "done":
            return
        self.phase = "done"
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
        if n.goal and n.goal.get("handle"):
            n.goal["handle"].cancel_goal_async()
            rclpy.spin_once(n, timeout_sec=0.5)
        n.say("Ctrl+C — 목표 취소, 멈춤", warn=True)
    finally:
        n.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
