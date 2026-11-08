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
class AC:
    def __init__(s, *a): pass
    def server_is_ready(s): return True
    def send_goal_async(s, g):
        h = Handle(); f = Fut(h); sent.append((g, h, f)); return f
mod("rclpy"); mod("rclpy.node", Node=Node); mod("rclpy.action", ActionClient=AC)
mod("rclpy.qos", DurabilityPolicy=types.SimpleNamespace(TRANSIENT_LOCAL=0), QoSProfile=lambda **k: None,
    ReliabilityPolicy=types.SimpleNamespace(RELIABLE=0))
mod("rclpy.signals", SignalHandlerOptions=types.SimpleNamespace(NO=0))
mod("nav2_msgs"); mod("nav2_msgs.action", NavigateToPose=types.SimpleNamespace(Goal=M))
for n, it in {"geometry_msgs.msg": ["Point", "PoseStamped", "PoseWithCovarianceStamped"], "nav_msgs.msg": ["OccupancyGrid", "Odometry"],
              "sensor_msgs.msg": ["PointCloud2"], "std_msgs.msg": ["ColorRGBA", "String"], "visualization_msgs.msg": ["Marker", "MarkerArray"]}.items():
    mod(n, **{i: M for i in it})
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

# A) 11시 통로: 관찰점에서 봐도 경계가 그대로 → 2단계로 경계 칸에 직접 → 그래도 그대로 → unseen
fr = [[{"x": -0.45, "y": 0.36, "size": 0.6, "cells": [(-0.45, 0.36)], "vx": 0.0, "vy": 0.41}]]
X = make(fr); sent.clear(); tick(X, 2); f = sent[-1][2]; f.cb(f)
ok("A1 처음엔 관찰점 (0.00, 0.41) 으로", goal_xy() == (0.0, 0.41))
answer(4); tick(X, 9)
ok("A2 관찰점에서 봐도 그대로 → 경계 칸 (−0.45, 0.36) 으로 직접", goal_xy() == (-0.45, 0.36) and X.gave_up == [])
answer(4); tick(X, 9)
ok("A3 직접 가서도 그대로 → unseen 포기", [g["reason"] for g in X.gave_up] == ["unseen"])
# B) 2단계 가는 중 Whisker 접촉 2번 → whisker 포기
fr = [[{"x": -0.45, "y": 0.36, "size": 0.6, "cells": [(-0.45, 0.36)], "vx": 0.0, "vy": 0.41}]]
X = make(fr); sent.clear(); tick(X, 2); f = sent[-1][2]; f.cb(f); answer(4); tick(X, 9); f = sent[-1][2]; f.cb(f)
for _ in range(2): X.on_sense(types.SimpleNamespace(data='{"ev": "lift"}'))
tick(X, 1); answer(5); tick(X, 2)
ok("B 직접 가다 Whisker 2번 → whisker 포기", [g["reason"] for g in X.gave_up] == ["whisker"])
# C) 1단계에서 경계가 사라지면(보임) 포기 없이 다음
fr = [[{"x": 1.0, "y": 0.5, "size": 0.5, "cells": [(1.0, 0.5)], "vx": 0.8, "vy": 0.4}]]
X = make(fr); sent.clear(); tick(X, 2); f = sent[-1][2]; f.cb(f); fr[0] = []; tick(X, 1); answer(5); tick(X, 2)
ok("C 가는 중 경계가 보이면 포기 없이 집으로", X.gave_up == [] and X.phase == "home")
# D) 취소에 Nav2 무응답 → 15 s 뒤 정리
fr = [[{"x": 1.0, "y": 0.5, "size": 0.5, "cells": [(1.0, 0.5)], "vx": 0.8, "vy": 0.4},
       {"x": -1.0, "y": 0.3, "size": 0.4, "cells": [(-1.0, 0.3)], "vx": -0.7, "vy": 0.3}]]
X = make(fr); sent.clear(); tick(X, 2); f = sent[-1][2]; f.cb(f); first = goal_xy(); tick(X, 50 + 20)
ok("D 45 s 진척 없음 → 취소 무응답 15 s → no_progress 포기, 다른 목표로", [g["reason"] for g in X.gave_up] == ["no_progress"] and goal_xy() != first)
print(f"\n{sum(R)}/{len(R)} 통과")
