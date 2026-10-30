#!/usr/bin/env python3
"""[Pi] 라이다로 실제 회전각 재기 (각도기 대신). 제자리 회전이면 방 전체가 그만큼 돌아 보인다는 점을 이용.

준비: 라이다만 켜기 (브리지는 끄기 — odom_probe 가 시리얼을 씀)
  ros2 launch sllidar_ros2 sllidar_c1_launch.py          (다른 창/tmux)
사용:
  python3 scripts/lidar_angle.py snap before      # 회전 전 스캔 저장 (10장 중앙값)
  python3 scripts/odom_probe.py spin ...          # 회전
  python3 scripts/lidar_angle.py snap after       # 회전 후 스캔 저장
  python3 scripts/lidar_angle.py compare          # 실제 회전각 (반시계 +), 0.5° 단위

원리: after 스캔을 한 칸(0.5°)씩 돌려가며 before 와 가장 잘 겹치는 칸 수를 찾는다.
로봇이 반시계로 θ 돌면, 같은 벽이 after 에서는 θ 만큼 작은 각도에 보인다.
주의: 한 바퀴(360°) 넘게 돌면 그 차이는 구분 못 함 (θ 와 θ±360 이 같아 보임) → 자이로 값으로 몇 바퀴인지 판단.
"""
import json
import math
import os
import statistics
import sys

REPO = os.path.realpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
SNAP = os.path.join(REPO, "logs", "scan_{}.json")


def snap(name, count=10):
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan

    rclpy.init()
    node = rclpy.create_node("lidar_angle")
    scans = []
    node.create_subscription(LaserScan, "scan", lambda m: scans.append(m), qos_profile_sensor_data)
    while len(scans) < count and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=2.0)
        if not scans:
            print("  /scan 기다리는 중... (라이다 켜져 있나?)")
    m = scans[0]
    n = len(m.ranges)
    med = []
    for i in range(n):   # 빔마다 10장의 중앙값 (튀는 값 제거)
        vals = [s.ranges[i] for s in scans if len(s.ranges) == n and math.isfinite(s.ranges[i])
                and m.range_min < s.ranges[i] < m.range_max]
        med.append(statistics.median(vals) if len(vals) >= count // 2 else None)
    os.makedirs(os.path.dirname(SNAP.format(name)), exist_ok=True)
    json.dump({"angle_min": m.angle_min, "inc": m.angle_increment, "ranges": med}, open(SNAP.format(name), "w"))
    valid = sum(r is not None for r in med)
    print(f"저장: {SNAP.format(name)}  ({n}빔 중 유효 {valid}, 간격 {math.degrees(m.angle_increment):.2f}°)")
    node.destroy_node(); rclpy.shutdown()


def compare():
    b = json.load(open(SNAP.format("before"))); a = json.load(open(SNAP.format("after")))
    rb, ra, inc = b["ranges"], a["ranges"], b["inc"]
    n = len(rb)
    if len(ra) != n:
        sys.exit("두 스캔의 빔 수가 다름")
    scores = []
    for k in range(n):   # after[i] 와 before[i + k] 비교
        diffs = [abs(ra[i] - rb[(i + k) % n]) for i in range(n) if ra[i] is not None and rb[(i + k) % n] is not None]
        if len(diffs) > n // 4:
            scores.append((statistics.median(diffs), k, len(diffs)))
    scores.sort()
    def deg(k):
        d = math.degrees(k * inc)
        return d - 360 if d > 180 else d
    best = scores[0]
    d = deg(best[1])
    alt = d + 360 if d < 0 else d - 360
    print(f"실제 회전 (반시계 +): {d:+.1f}°  또는 {alt:+.1f}°  ← 둘 중 자이로 방향·크기와 맞는 쪽 (겹침 오차 중앙값 {best[0] * 100:.1f} cm, 비교 빔 {best[2]})")
    print("다음 후보:", ", ".join(f"{deg(k):+.1f}° ({s * 100:.1f} cm)" for s, k, _ in scores[1:4]))
    print("→ 오차가 수 cm 이하면 믿을 만함. 360° 넘게 돌았으면 자이로로 바퀴 수를 더할 것")


if __name__ == "__main__":
    if len(sys.argv) >= 3 and sys.argv[1] == "snap":
        snap(sys.argv[2])
    elif len(sys.argv) >= 2 and sys.argv[1] == "compare":
        compare()
    else:
        print(__doc__)
