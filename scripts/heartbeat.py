#!/usr/bin/env python3
"""heartbeat.py — 로그 2층 "심박": 1분마다 로봇 상태 한 줄 (docs/design/logging.md).

  [Pi] source ~/scr_ws/install/setup.bash && python3 scripts/heartbeat.py            # 60초마다
       python3 scripts/heartbeat.py --period 10                                       # 시험용 10초
  → ~/scr_logs/<날짜>/heartbeat.jsonl  (한 줄 = 한 번)

  {"t":..., "v":11.62, "v_min":11.48, "x":0.41, "y":-0.12, "frame":"map", "cpu":44.8, "temp":51.0,
   "mem_mb":812, "procs":{"slam":9.1, "controller":12.3, ...}}

  - v      : 그 사이 /battery_state 평균 (모터가 당길 때 순간 처짐 제외용), v_min: 그 사이 최저
  - x, y   : map 좌표 (slam_toolbox /pose 또는 /amcl_pose, 없으면 /odom), 못 찾으면 생략
  - cpu    : 전체 CPU % (코어 4개 합쳐 100%)
  - procs  : 프로세스 묶음별 CPU % (코어 하나 = 100%). SLAM ↔ AMCL 전후 비교용
  - 브리지가 안 떠 있어도 돈다 (v·위치만 빠짐). 배터리 실사용 시간 측정 = 이걸 켜두고 전압 곡선 보기.
"""
import argparse
import json
import os
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry

GROUPS = [  # (이름, 명령줄에 들어 있는 글자) — 위에서부터 먼저 맞는 것
    ("bridge", "ugv02_bridge"), ("lidar", "sllidar"), ("slam", "slam_toolbox"), ("amcl", "amcl"),
    ("controller", "controller_server"), ("planner", "planner_server"), ("explore", "explore"),
    # 11-05: 묶음 "nav2_other" 가 코어 하나를 거의 다 씀 (85~95%) → 누군지 보려고 하나씩 나눔
    ("bt_navigator", "bt_navigator"), ("behavior", "behavior_server"), ("velocity_smoother", "velocity_smoother"),
    ("smoother", "smoother_server"), ("collision_monitor", "collision_monitor"), ("lifecycle", "lifecycle_manager"),
    ("waypoint", "waypoint_follower"), ("route", "route_server"), ("docking", "opennav_docking"),
    ("map_server", "map_server"),
    ("tof", "tof_node"), ("heartbeat", "heartbeat.py"), ("ros_cli", "ros2"),
]
TICK = os.sysconf("SC_CLK_TCK")


def cpu_total():
    with open("/proc/stat") as f:
        v = list(map(int, f.readline().split()[1:]))
    return sum(v), v[3] + v[4]  # 전체, 쉼(idle+iowait)


def proc_times():
    """pid → (묶음 이름, utime+stime 틱)"""
    out = {}
    for d in os.listdir("/proc"):
        if not d.isdigit():
            continue
        try:
            cmd = open(f"/proc/{d}/cmdline", "rb").read().replace(b"\0", b" ").decode(errors="replace")
            st = open(f"/proc/{d}/stat").read().rsplit(")", 1)[1].split()
        except OSError:
            continue
        group = next((g for g, key in GROUPS if key in cmd), None)
        if group:
            out[d] = (group, int(st[11]) + int(st[12]))
    return out


def temp_c():
    try:
        return int(open("/sys/class/thermal/thermal_zone0/temp").read()) / 1000
    except OSError:
        return None


def mem_used_mb():
    info = {}
    for line in open("/proc/meminfo"):
        k, v = line.split(":")
        info[k] = int(v.split()[0])
    return (info["MemTotal"] - info["MemAvailable"]) // 1024


class Heartbeat(Node):
    def __init__(self, period):
        super().__init__("heartbeat")
        self.period = period
        self.volts = []
        self.create_subscription(BatteryState, "battery_state", lambda m: self.volts.append(m.voltage), 10)
        # 위치: TF 를 통째로 듣던 것(11-05 주행 중 10% CPU) 대신 위치 토픽만. 지도 위치가 있으면 그걸, 없으면 odom.
        self.poses = {}
        for topic, frame, typ in (("pose", "map", PoseWithCovarianceStamped),       # slam_toolbox
                                  ("amcl_pose", "map", PoseWithCovarianceStamped),  # AMCL
                                  ("odom", "odom", Odometry)):                     # 브리지 (20 Hz)
            self.create_subscription(typ, topic, lambda m, f=frame: self.poses.__setitem__(f, m.pose.pose.position), 10)
        self.prev_cpu = cpu_total()
        self.prev_procs = proc_times()
        self.prev_t = time.time()
        self.create_timer(period, self.beat)
        self.get_logger().info(f"heartbeat every {period:.0f} s → {self.path()}")

    def path(self):
        d = Path.home() / "scr_logs" / time.strftime("%Y-%m-%d")
        d.mkdir(parents=True, exist_ok=True)
        return d / "heartbeat.jsonl"

    def pose(self):
        for frame in ("map", "odom"):
            if frame in self.poses:
                p = self.poses[frame]
                return frame, p.x, p.y
        return None

    def beat(self):
        now = time.time()
        total, idle = cpu_total()
        dt_total = total - self.prev_cpu[0]
        cpu = 100.0 * (1 - (idle - self.prev_cpu[1]) / dt_total) if dt_total else None
        procs_now = proc_times()
        groups = {}
        wall = now - self.prev_t
        for pid, (g, ticks) in procs_now.items():
            if pid in self.prev_procs:
                groups[g] = groups.get(g, 0.0) + (ticks - self.prev_procs[pid][1]) / TICK / wall * 100
        rec = {"t": round(now, 1)}
        if self.volts:
            rec["v"] = round(sum(self.volts) / len(self.volts), 2)
            rec["v_min"] = round(min(self.volts), 2)
        p = self.pose()
        if p:
            rec["frame"], rec["x"], rec["y"] = p[0], round(p[1], 2), round(p[2], 2)
        rec["cpu"] = round(cpu, 1) if cpu is not None else None
        rec["temp"] = temp_c()
        rec["mem_mb"] = mem_used_mb()
        rec["procs"] = {g: round(c, 1) for g, c in sorted(groups.items(), key=lambda kv: -kv[1]) if c >= 0.1}
        with open(self.path(), "a") as f:
            f.write(json.dumps(rec, ensure_ascii=False) + "\n")
        self.get_logger().info(json.dumps(rec, ensure_ascii=False))
        self.volts.clear()
        self.prev_cpu, self.prev_procs, self.prev_t = (total, idle), procs_now, now


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--period", type=float, default=60.0, help="초 (기본 60)")
    a = ap.parse_args()
    rclpy.init()
    node = Heartbeat(a.period)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
