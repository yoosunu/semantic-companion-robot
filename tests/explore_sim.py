#!/usr/bin/env python3
"""scr_explore 흉내 시험 (로봇·Nav2 없이): 가짜 rclpy·액션 서버. 경계는 손으로 넣음.
  python3 tests/explore_sim.py
"""
import os, sys, types, math
T = [1000.0]
import time as _time
_time.time = lambda: T[0]
def mod(n, **kw):
    m = types.ModuleType(n); m.__dict__.update(kw); sys.modules[n] = m; return m
class Node:
    def __init__(s, n): s.logs = []
    def create_subscription(s, *a, **k): pass
    def create_publisher(s, *a, **k): return types.SimpleNamespace(publish=lambda m: None)
    def create_timer(s, *a): pass
    def get_logger(s):
        L = s.logs; f = lambda x: L.append(x)
        return types.SimpleNamespace(info=f, warn=f, error=f)
    def get_clock(s): return types.SimpleNamespace(now=lambda: types.SimpleNamespace(to_msg=lambda: None))
class M:
    DELETEALL = ADD = POINTS = SPHERE = TEXT_VIEW_FACING = 0
    def __init__(s, *a, **k):
        s.__dict__.update(k)
        s.pose = types.SimpleNamespace(position=types.SimpleNamespace(), orientation=types.SimpleNamespace())
        s.header = types.SimpleNamespace()
class Fut:
    def __init__(s, r): s.r, s.cb = r, None
    def result(s): return s.r
    def add_done_callback(s, cb): s.cb = cb
class Handle:
    accepted = True
    def __init__(s): s.res = Fut(None); s.cancelled = 0
    def get_result_async(s): return s.res
    def cancel_goal_async(s): s.cancelled += 1
sent = []
plans = []                                   # 플래너에게 길을 물은 것 [(goal, handle, fut, answered)]
AUTO = [None]                                # 길 자동 답 (점 목록). None = 답 안 함
class AC:
    def __init__(s, node=None, typ=None, name=""): s.name = name
    def server_is_ready(s): return True
    def send_goal_async(s, g):
        h = Handle(); f = Fut(h)
        (plans if s.name == "compute_path_to_pose" else sent).append([g, h, f, False] if s.name == "compute_path_to_pose" else (g, h, f))
        return f
def give_path(pts, status=4):
    pl = plans[-1]; pl[3] = True; g, h, f, _ = pl
    if f.cb: f.cb(f)
    poses = [types.SimpleNamespace(pose=types.SimpleNamespace(position=types.SimpleNamespace(x=x, y=y))) for x, y in pts]
    h.res.r = types.SimpleNamespace(status=status, result=types.SimpleNamespace(path=types.SimpleNamespace(poses=poses)))
    if h.res.cb: h.res.cb(h.res)
def line(a, b, n=None):
    n = n or max(2, int(math.hypot(b[0] - a[0], b[1] - a[1]) / 0.05) + 1)
    return [(a[0] + (b[0] - a[0]) * k / (n - 1), a[1] + (b[1] - a[1]) * k / (n - 1)) for k in range(n)]
mod("rclpy"); mod("rclpy.node", Node=Node); mod("rclpy.action", ActionClient=AC)
mod("rclpy.qos", DurabilityPolicy=types.SimpleNamespace(TRANSIENT_LOCAL=0), QoSProfile=lambda **k: None,
    ReliabilityPolicy=types.SimpleNamespace(RELIABLE=0))
mod("rclpy.signals", SignalHandlerOptions=types.SimpleNamespace(NO=0))
mod("nav2_msgs"); mod("nav2_msgs.action", NavigateToPose=types.SimpleNamespace(Goal=M), ComputePathToPose=types.SimpleNamespace(Goal=M))
class TW:
    def __init__(s): s.header = types.SimpleNamespace(); s.twist = types.SimpleNamespace(linear=types.SimpleNamespace(), angular=types.SimpleNamespace())
for n, it in {"geometry_msgs.msg": ["Point", "PoseStamped", "PoseWithCovarianceStamped"], "nav_msgs.msg": ["OccupancyGrid", "Odometry"],
              "sensor_msgs.msg": ["PointCloud2"], "std_msgs.msg": ["ColorRGBA", "String"], "visualization_msgs.msg": ["Marker", "MarkerArray"]}.items():
    mod(n, **{i: M for i in it})
sys.modules["geometry_msgs.msg"].TwistStamped = TW
for n in ("geometry_msgs", "nav_msgs", "sensor_msgs", "std_msgs", "visualization_msgs"): mod(n)
sys.modules["visualization_msgs.msg"].MarkerArray = lambda: types.SimpleNamespace(markers=[])
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "scripts"))
import scr_explore as E
E.ntfy = lambda *a, **k: None

def make(frontiers):
    X = E.Explorer("t"); X.publish_markers = lambda: None
    X.odom_hist.append((0, 0.0, 0.0, 0.0)); X.grid = X.cmap = object()
    X.find_frontiers = lambda: frontiers[0]
    return X
def tick(X, n=1):
    for _ in range(n): T[0] += 1; X.step()
def answer(status):
    g, h, f = sent[-1]
    if f.cb: f.cb(f)
    h.res.r = types.SimpleNamespace(status=status); h.res.cb(h.res)
def goal_xy(): p = sent[-1][0].pose.pose.position; return round(p.x, 2), round(p.y, 2)
R = []
def ok(title, cond):
    R.append(cond); print(f"{'✓' if cond else '✗'} {title}")

cmds = []
def make(frontiers):
    X = E.Explorer("t"); X.publish_markers = lambda: None
    X.odom_hist.append((0, 0.0, 0.0, 0.0)); X.grid = X.cmap = object()
    X.find_frontiers = lambda: frontiers[0]
    X.cmd_pub = types.SimpleNamespace(publish=lambda m: cmds.append((round(m.twist.linear.x, 2), round(m.twist.angular.z, 2))))
    return X
def move(X, x, y, yaw=0.0): X.odom_hist.append((0, x, y, yaw))
def ptick(X, n):
    for _ in range(n):
        T[0] += 0.1; X.probe_tick()
        if plans and not plans[-1][3] and AUTO[0] is not None and X.probe:
            give_path(AUTO[0])
F = lambda: [[{"x": -0.45, "y": 0.36, "size": 0.6, "cells": [(-0.45, 0.36)], "vx": 0.0, "vy": 0.41, "ax": -0.25, "ay": 0.36}]]
reasons = lambda X: [g["reason"] for g in X.gave_up]

AUTO[0] = line((-0.25, 0.36), (-0.45, 0.36))              # 11시: 경계 옆 칸 → 경계, 곧은 길
# A) 11시 통로 (11-08 21:15): 관찰점 → 경계 옆 싼 칸 → 더듬기 → 그래도 그대로 → unseen
fr = F(); X = make(fr); sent.clear(); tick(X, 2); f = sent[-1][2]; f.cb(f)
ok("A1 처음엔 관찰점 (0.00, 0.41) 으로", goal_xy() == (0.0, 0.41))
answer(4); tick(X, 9)
ok("A2 그대로 → 경계 칸 아닌 '경계 옆 싼 칸' (−0.25, 0.36) 으로", goal_xy() == (-0.25, 0.36) and X.gave_up == [])
move(X, -0.25, 0.36); answer(4)
for _ in range(12):
    tick(X, 1)
    if X.probe: break
ok("A3 거기서도 그대로 → 더듬기 시작 (Nav2 목표 없음)", X.probe is not None and X.goal is None)
cmds.clear(); ptick(X, 12)
ok("A4 더듬기 전 1.5 s 는 조용 (Nav2 감속 기다림)", cmds == [])
ptick(X, 10)
ok("A5 플래너 길 받음 → 길이 뒤쪽 → 먼저 제자리에서 왼쪽으로 돎", len(plans) >= 1 and X.probe["path"] is not None and bool(cmds) and cmds[-1][0] == 0.0 and cmds[-1][1] > 0)
move(X, -0.25, 0.36, math.pi); cmds.clear(); ptick(X, 3)
ok("A6 경계 쪽을 봄 → 천천히 직진 0.07", cmds and cmds[-1][0] == 0.07)
move(X, -0.42, 0.36, math.pi); ptick(X, 2)
ok("A7 길 끝 6 cm 안 → 더듬기 끝, 정지", X.probe is None and cmds[-1] == (0.0, 0.0))
tick(X, 9)
ok("A8 더듬기까지 했는데도 그대로 → unseen 포기", reasons(X) == ["unseen"])

# B) 더듬다 Whisker 2번 → whisker 포기. 빼는 동안은 명령 안 냄
fr = F(); X = make(fr); sent.clear(); tick(X, 2); f = sent[-1][2]; f.cb(f); answer(4); tick(X, 9)
move(X, -0.25, 0.36, math.pi); answer(4); tick(X, 9); ptick(X, 20)
X.on_sense(types.SimpleNamespace(data='{"ev": "bump"}')); X.on_wstate(types.SimpleNamespace(data='{"state": "back"}'))
cmds.clear(); ptick(X, 10)
ok("B1 Whisker 가 빼는 중 → 더듬기는 명령 안 냄", cmds == [] and X.probe is not None)
X.on_wstate(types.SimpleNamespace(data='{"state": "drive"}')); ptick(X, 15)
ok("B2 돌려받고 1 s 뒤 다시 더듬음", cmds != [] and X.probe is not None)
X.on_sense(types.SimpleNamespace(data='{"ev": "stuck"}')); ptick(X, 1)
ok("B3 두 번째 접촉 → whisker 포기, 정지", reasons(X) == ["whisker"] and X.probe is None and cmds[-1] == (0.0, 0.0))

# E) 2단계에서 Nav2 멈칫 15 s → 45 s 기다리지 않고 더듬기 (포기 아님)
fr = F(); X = make(fr); sent.clear(); tick(X, 2); f = sent[-1][2]; f.cb(f); answer(4); tick(X, 9); f = sent[-1][2]; f.cb(f)
tick(X, 16)
ok("E1 15 s 못 다가감 → 취소 (stall)", X.goal is not None and X.goal.get("cancel_reason") == "stall")
answer(5); tick(X, 1)
ok("E2 포기 없이 바로 더듬기", X.gave_up == [] and X.probe is not None)

# F) 싼 칸이 지금 자리보다 별로 안 가까움 → Nav2 생략, 바로 더듬기
fr = [[{"x": -0.45, "y": 0.36, "size": 0.6, "cells": [(-0.45, 0.36)], "vx": -0.1, "vy": 0.36, "ax": -0.12, "ay": 0.36}]]
X = make(fr); move(X, -0.1, 0.36); sent.clear(); tick(X, 2); f = sent[-1][2]; f.cb(f); n0 = len(sent); answer(4); tick(X, 9)
ok("F 싼 칸 = 사실상 지금 자리 → Nav2 안 보내고 더듬기", len(sent) == n0 and X.probe is not None)

# G) 더듬다 경계가 보이면 → 포기 없이 다음
fr = F(); X = make(fr); sent.clear(); tick(X, 2); f = sent[-1][2]; f.cb(f); answer(4); tick(X, 9)
move(X, -0.25, 0.36, math.pi); answer(4); tick(X, 9); fr[0] = []; tick(X, 1)
ok("G 더듬다 경계가 보임 → 포기 없이 끝", X.probe is None and X.gave_up == [])

W = lambda X, st: X.on_wstate(types.SimpleNamespace(data='{"state": "%s"}' % st))
# H) 1단계 가는 중 Whisker 가 맡음 → Nav2 목표 내려놓음(포기 아님) → 돌려주면 1 s 뒤 같은 목표 다시, 접촉 수 이어짐
fr = [[{"x": 1.0, "y": 0.5, "size": 0.5, "cells": [(1.0, 0.5)], "vx": 0.8, "vy": 0.4}]]
X = make(fr); sent.clear(); tick(X, 2); f = sent[-1][2]; f.cb(f); h0 = sent[-1][1]
X.on_sense(types.SimpleNamespace(data='{"ev": "bump"}')); W(X, "stop"); W(X, "back")
ok("H1 Whisker 가 맡자마자 Nav2 목표 취소", h0.cancelled == 1)
answer(5); n0 = len(sent); tick(X, 60)
ok("H2 빼기·갇힘 60 s 동안 새 목표 없음, 포기 없음 (우리 시계도 쉼)", len(sent) == n0 and X.gave_up == [])
W(X, "drive"); tick(X, 2)
ok("H3 돌려받고 같은 목표 다시 보냄, 접촉 1 이어짐", len(sent) == n0 + 1 and goal_xy() == (0.8, 0.4) and X.goal["contacts"] == 1)
f = sent[-1][2]; f.cb(f); tick(X, 30)
ok("H4 다시 보낸 뒤 30 s 는 45 s 진척 검사에 안 걸림", X.gave_up == [])
X.on_sense(types.SimpleNamespace(data='{"ev": "bump"}')); W(X, "stop"); answer(5); W(X, "back"); tick(X, 3); W(X, "drive"); tick(X, 2)
ok("H5 두 번째 접촉 → 돌려받은 뒤 whisker 포기 (다시 안 보냄)", reasons(X) == ["whisker"])
# I) 2단계(경계 옆) 가는 중 Whisker → 15 s 멈칫 검사가 빼기 시간을 세지 않음
fr = F(); X = make(fr); sent.clear(); tick(X, 2); f = sent[-1][2]; f.cb(f); answer(4); tick(X, 9); f = sent[-1][2]; f.cb(f)
tick(X, 5); W(X, "back"); answer(5); tick(X, 20); W(X, "drive"); tick(X, 2); f = sent[-1][2]; f.cb(f); tick(X, 10)
ok("I 빼기 20 s 뒤 다시 2단계, 10 s 지나도 멈칫 아님 (더듬기·포기 없음)", X.goal is not None and X.goal["stage"] == 2 and X.probe is None and X.gave_up == [])
# J) 더듬기 중 Whisker 갇힘 대기 40 s → 더듬기 시계(25 s)가 안 끝남
fr = F(); X = make(fr); sent.clear(); tick(X, 2); f = sent[-1][2]; f.cb(f); answer(4); tick(X, 9)
move(X, -0.25, 0.36, math.pi); answer(4)
for _ in range(12):
    tick(X, 1)
    if X.probe: break
ptick(X, 20); W(X, "trapped"); ptick(X, 400); W(X, "drive"); ptick(X, 15)
ok("J 갇힘 대기 40 s 뒤에도 더듬기 계속 (시간 끝 아님)", X.probe is not None and X.reached == [])
# K) 집 가는 중 Whisker → 집 가기 멈춤 → 돌려주면 다시 집으로 (끝내지 않음)
fr = [[]]; X = make(fr); sent.clear(); tick(X, 2); f = sent[-1][2]; f.cb(f); h0 = sent[-1][1]
W(X, "back"); answer(5); tick(X, 3)
ok("K1 집 가다 Whisker → 끝내지 않음", X.phase == "home" and h0.cancelled == 1)
W(X, "drive"); tick(X, 2)
ok("K2 돌려받고 다시 집으로", len(sent) == 2 and goal_xy() == (0.0, 0.0))

# L) 관찰점 없음 기록 (11-08 22:14 11시): 초반에 잠깐 없었던 기록이 남아 있다가 가는 중 잠깐 사라지면 바로 포기했음
fr = [[{"x": -0.77, "y": 0.30, "size": 0.6, "cells": [(-0.77, 0.30)], "vx": None, "vy": None}]]
X = make(fr); sent.clear(); tick(X, 3)
fr[0] = [{"x": -0.77, "y": 0.30, "size": 0.6, "cells": [(-0.77, 0.30)], "vx": 0.03, "vy": 0.65}]; tick(X, 2)
ok("L1 관찰점이 생기면 그리로 출발, 기록 지워짐", goal_xy() == (0.03, 0.65) and X.no_view == {})
f = sent[-1][2]; f.cb(f)
fr[0] = [{"x": -0.77, "y": 0.30, "size": 0.6, "cells": [(-0.77, 0.30)], "vx": None, "vy": None}]
for i in range(40): move(X, 0.0, 0.016 * i); tick(X, 1)                  # 천천히 다가감 (진척 있음)
ok("L2 가는 중 관찰점이 잠깐 사라져도 (40 s) 포기 안 함", X.gave_up == [] and X.goal is not None)
fr[0] = [{"x": -0.77, "y": 0.30, "size": 0.6, "cells": [(-0.77, 0.30)], "vx": 0.03, "vy": 0.65}]
answer(4); tick(X, 9)
ok("L3 도착 후 그대로 → 2단계로 이어감 (포기 없음)", X.gave_up == [] and (X.goal is not None or X.probe is not None))

# M) 2시 (11-08 22:14): 경계는 북쪽인데 사이에 벽 → 플래너 길(서쪽으로 돌아감)을 따라감, 직진 안 함
fr = [[{"x": 1.68, "y": 0.30, "size": 0.5, "cells": [(1.68, 0.30)], "vx": 1.27, "vy": 0.30, "ax": 1.48, "ay": 0.45}]]
X = make(fr); move(X, 1.50, -0.47, math.pi / 2); X.approached.append((1.68, 0.30)); tick(X, 1)
AUTO[0] = line((1.50, -0.47), (1.30, -0.47)) + line((1.30, -0.42), (1.30, 0.30)) + line((1.35, 0.30), (1.68, 0.30))
cmds.clear(); ptick(X, 20)
ok("M 벽 너머 경계 → 직진(북쪽) 말고 길 따라 서쪽으로 먼저 돎", cmds and cmds[-1][0] == 0.0 and cmds[-1][1] > 0)
# N) 플래너가 길 없음 → no_path / 너무 긴 길 → far
fr = F(); X = make(fr); move(X, -0.25, 0.36); X.approached.append((-0.45, 0.36)); tick(X, 1); AUTO[0] = None; ptick(X, 20); give_path([])
ok("N1 플래너 길 없음 → no_path 포기", reasons(X) == ["no_path"] and X.probe is None)
fr = F(); X = make(fr); move(X, -0.25, 0.36); X.approached.append((-0.45, 0.36)); tick(X, 1); ptick(X, 20)
give_path(line((-0.25, 0.36), (-0.25, 1.86)) + line((-0.25, 1.86), (-0.45, 0.36)))
ok("N2 길이 2.5 m 넘음 → far 포기 (더듬기 아님)", reasons(X) == ["far"])
# O) 더듬다 Whisker 가 빼면 → 돌려받은 자리에서 길을 새로 받음
AUTO[0] = line((-0.25, 0.36), (-0.45, 0.36))
fr = F(); X = make(fr); move(X, -0.25, 0.36); X.approached.append((-0.45, 0.36)); tick(X, 1); ptick(X, 20); n0 = len(plans)
X.on_sense(types.SimpleNamespace(data='{"ev": "bump"}')); W(X, "back"); ptick(X, 30); move(X, -0.15, 0.36); W(X, "drive"); ptick(X, 15)
ok("O 빼고 돌려받음 → 길 다시 받음, 접촉 1 이어짐", len(plans) == n0 + 1 and X.probe and X.probe["contacts"] == 1)
# P) 2단계인데 경계에서 멀리(0.8 m 밖) 멈칫 → 더듬지 않고 Nav2 에 계속 맡김
fr = [[{"x": 1.68, "y": 0.30, "size": 0.5, "cells": [(1.68, 0.30)], "vx": 1.27, "vy": 0.30, "ax": 1.48, "ay": 0.45}]]
X = make(fr); move(X, 1.50, -0.60); X.viewed.append((1.68, 0.30)); sent.clear(); tick(X, 1); f = sent[-1][2]; f.cb(f); tick(X, 20)
ok("P 멀리서 20 s 멈칫 → 더듬기 안 함 (45 s 규칙까지 Nav2)", X.probe is None and X.goal is not None and X.gave_up == [])

# Q) 더듬는 중 바로 옆 경계가 관찰점 없음 → 따로 포기하지 않음 (11-08 22:54 11시 포기 2개)
fr = F(); X = make(fr); move(X, -0.25, 0.36); X.approached.append((-0.45, 0.36)); tick(X, 1)
fr[0] = [{"x": -0.55, "y": 0.30, "size": 0.5, "cells": [(-0.55, 0.30), (-0.45, 0.36)], "vx": None, "vy": None}]; AUTO[0] = None; tick(X, 40)
ok("Q 더듬는 중 옆 경계(10 cm) 관찰점 40 s 없음 → 포기 안 함", X.gave_up == [] and X.probe is not None)

# R) 거실 (11-08 23:38): 관찰점 없는 먼 경계 → 다른 곳 보러 다니는 동안은 포기 안 함, 할 일 없이 30 s 면 이유와 함께 포기
fr = [[{"x": 1.0, "y": 0.5, "size": 0.5, "cells": [(1.0, 0.5)], "vx": 0.8, "vy": 0.4},
       {"x": -1.3, "y": 2.0, "size": 3.0, "cells": [(-1.3, 2.0)], "vx": None, "vy": None, "why": "가장 가까운 안전 칸 1.40 m"}]]
X = make(fr); sent.clear(); tick(X, 2); f = sent[-1][2]; f.cb(f)
for i in range(60): move(X, 0.013 * i, 0.006 * i); tick(X, 1)
ok("R1 다른 경계로 가는 60 s 동안 먼 경계 포기 안 함", X.gave_up == [])
fr[0] = fr[0][1:]; answer(5); tick(X, 25)
ok("R2 할 일 없이 25 s → 아직 보류", X.gave_up == [])
tick(X, 10)
ok("R3 할 일 없이 30 s 넘음 → no_view 포기 (이유 로그)", reasons(X) == ["no_view"] and any("1.40 m" in l for l in X.logs))
# S) 1단계: 관찰점 37 cm 앞에서 MPPI 멈칫 15 s → 본 걸로 (45 s no_progress 포기 아님)
fr = [[{"x": 5.14, "y": 1.04, "size": 0.5, "cells": [(5.14, 1.04)], "vx": 4.69, "vy": 0.79}]]
X = make(fr); move(X, 4.64, 0.42); sent.clear(); tick(X, 2); f = sent[-1][2]; f.cb(f); tick(X, 16)
ok("S1 관찰점 앞 15 s 멈칫 → 취소 (near)", X.goal and X.goal.get("cancel_reason") == "near")
answer(5); tick(X, 9)
ok("S2 포기 없이 '봤음' → 2단계로", X.gave_up == [] and X.near(5.14, 1.04, X.viewed) and (X.goal is not None or X.probe is not None))

# C) 1단계에서 경계가 사라지면(보임) 포기 없이 다음
fr = [[{"x": 1.0, "y": 0.5, "size": 0.5, "cells": [(1.0, 0.5)], "vx": 0.8, "vy": 0.4}]]
X = make(fr); sent.clear(); tick(X, 2); f = sent[-1][2]; f.cb(f); fr[0] = []; tick(X, 1); answer(5); tick(X, 2)
ok("C 가는 중 경계가 보이면 포기 없이 집으로", X.gave_up == [] and X.phase == "home")
# D) 1단계 45 s 진척 없음 + 취소 무응답 → 15 s 뒤 정리
fr = [[{"x": 1.0, "y": 0.5, "size": 0.5, "cells": [(1.0, 0.5)], "vx": 0.8, "vy": 0.4},
       {"x": -1.0, "y": 0.3, "size": 0.4, "cells": [(-1.0, 0.3)], "vx": -0.7, "vy": 0.3}]]
X = make(fr); sent.clear(); tick(X, 2); f = sent[-1][2]; f.cb(f); first = goal_xy(); tick(X, 50 + 20)
ok("D 1단계는 45 s 그대로 → 취소 무응답 15 s → no_progress 포기, 다른 목표로", reasons(X) == ["no_progress"] and goal_xy() != first)
print(f"\n{sum(R)}/{len(R)} 통과")
