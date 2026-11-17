#!/usr/bin/env python3
"""[Pi] 팬틸트 머리가 LiDAR 를 얼마나 가리나 (11-17). 고개를 돌려 가며 "가까운 빔" 을 기록.

  11-17 결과: 고정대가 뒤 +160~−157° 를 2~4 cm, 서보선이 3~17 cm → robot.launch 에서 0.18 m 안쪽 거름 (/scan).
        그래서 이 도구는 거르기 전 /scan_raw 를 봄
  준비: 로봇만 켜기 (브리지 + LiDAR: ros2 launch scr_bringup robot.launch.py). look·executive 는 끄기 (고개를 이 도구가 씀)
        주변 0.5 m 안에 물건 없게 (가까운 빔 = 로봇 자신이어야 함)
  사용: python3 tools/bench/lidar_occlusion.py                     팬 −180~180 (15° 간격, 틸트 10) + 틸트 −30~90 (팬 0)  ≈ 2분
        python3 tools/bench/lidar_occlusion.py --step 30           빨리 (30° 간격)
        python3 tools/bench/lidar_occlusion.py --topic scan --step 45   거른 /scan 확인 (상자 필터 뒤 가까운 빔 0)
  판정: 빔 거리 NEAR (0.40 m) 안 = 가까움
        모든 자세에서 가까움 = 차체·판 (고개와 무관) / 어떤 자세에서만 = 머리가 가림
  각도: 로봇 기준 (0 = 앞, + = 왼쪽, ±180 = 뒤). LiDAR 0° 는 로봇 뒤 (URDF laser_yaw = pi) → 로봇각 = LiDAR각 + 180
        팬은 /pantilt/cmd 기준 (+ = 오른쪽)
  기록: ~/scr_logs/lidar_occlusion/<시각>.json (자세별 가까운 빔 [로봇각°, 거리 m])
"""
import argparse
import json
import math
import os
import statistics as st
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray

NEAR = 0.40
SPD, ACC = 512, 10              # 45°/s 쯤, 조용히
TILT_HOME = 10.0


def wrap(a):
    return (a + 180.0) % 360.0 - 180.0


def segments(angles):
    """정렬된 로봇각 목록 → [(시작, 끝)] (1.5° 넘게 떨어지면 끊음, ±180 이어붙임)"""
    if not angles:
        return []
    a = sorted(angles)
    segs, s, p = [], a[0], a[0]
    for x in a[1:]:
        if x - p > 1.5:
            segs.append((s, p))
            s = x
        p = x
    segs.append((s, p))
    if len(segs) > 1 and segs[0][0] <= -178.5 and segs[-1][1] >= 178.5:
        segs = [(segs[-1][0], segs[0][1])] + segs[1:-1]          # 뒤(±180)를 걸친 구간
    return segs


def fmt(segs):
    return ", ".join(f"{a:+.0f}~{b:+.0f}" for a, b in segs) or "없음"


class Probe(Node):
    def __init__(self, topic):
        super().__init__("lidar_occlusion")
        self.scans = []
        self.create_subscription(LaserScan, topic, lambda m: self.scans.append(m), qos_profile_sensor_data)
        self.pub = self.create_publisher(Float32MultiArray, "pantilt/cmd", 10)

    def wait(self, secs):
        end = time.time() + secs
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def pose(self, pan, tilt, settle):
        for _ in range(3):
            self.pub.publish(Float32MultiArray(data=[float(pan), float(tilt), float(SPD), float(ACC)]))
            self.wait(0.1)
        self.wait(settle)
        self.scans = []
        while len(self.scans) < 6:
            rclpy.spin_once(self, timeout_sec=0.2)
        m = self.scans[0]
        n = min(len(s.ranges) for s in self.scans)
        near = []
        for i in range(n):
            r = [s.ranges[i] for s in self.scans if math.isfinite(s.ranges[i]) and s.ranges[i] > 0.02]
            if len(r) >= 3 and st.median(r) < NEAR:
                ang = wrap(math.degrees(m.angle_min + i * m.angle_increment) + 180.0)
                near.append((round(ang, 1), round(st.median(r), 3)))
        return near


def main():
    a = argparse.ArgumentParser()
    a.add_argument("--step", type=float, default=15.0)
    a.add_argument("--topic", default="scan_raw", help="scan = 거른 뒤 확인 (가까운 빔이 0 이어야)")
    o = a.parse_args()
    rclpy.init()
    p = Probe(o.topic)
    print("스캔 기다리는 중...", flush=True)
    while not p.scans:
        rclpy.spin_once(p, timeout_sec=0.5)
    poses = [(pan, TILT_HOME) for pan in [-180.0 + k * o.step for k in range(int(360 / o.step) + 1)]]
    poses += [(0.0, t) for t in (-30.0, -5.0, 45.0, 90.0)]
    res, prev = [], None
    for pan, tilt in poses:
        d = 360.0 if prev is None else max(abs(pan - prev[0]), abs(tilt - prev[1]))
        near = p.pose(pan, tilt, settle=d / 45.0 + 0.8)
        prev = (pan, tilt)
        res.append({"pan": pan, "tilt": tilt, "near": near})
        print(f"팬 {pan:+5.0f} 틸트 {tilt:+3.0f} | 가까운 빔 {len(near):3d} | {fmt(segments([x[0] for x in near]))}", flush=True)
    p.pose(0.0, TILT_HOME, settle=4.0)

    sets = [set(x[0] for x in r["near"]) for r in res]
    always = set.intersection(*sets) if sets else set()
    head = set.union(*sets) - always if sets else set()
    print(f"\n늘 가까움 (차체·판, 고개 무관): {fmt(segments(list(always)))}")
    print(f"머리가 가리는 곳 (어떤 자세에서만, 전부 합침): {fmt(segments(list(head)))}  = {len(head)} 빔")
    worst = max(res, key=lambda r: len(r["near"]))
    print(f"가장 많이 가린 자세: 팬 {worst['pan']:+.0f} 틸트 {worst['tilt']:+.0f} ({len(worst['near'])} 빔)")
    out = os.path.expanduser("~/scr_logs/lidar_occlusion")
    os.makedirs(out, exist_ok=True)
    path = os.path.join(out, time.strftime("%Y%m%d_%H%M%S") + ".json")
    with open(path, "w") as f:
        json.dump({"near_m": NEAR, "poses": res, "always": sorted(always), "head": sorted(head)}, f)
    print(f"기록: {path}")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
