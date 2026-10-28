#!/usr/bin/env python3
"""wander.py — 실물 첫 SLAM 맛보기용 '혼자 돌아다니기' (10-29 새벽).

하는 일
- /scan 을 보고: 앞이 비었으면 천천히 전진, 막히면 더 넓은 쪽으로 제자리 회전.
- /cmd_vel 에 TwistStamped 로 보냄 (ugv02_bridge 가 받음, 0.5 s 명령 끊기면 브리지가 정지).
- SAVE_EVERY 초마다 지도 저장 (~/maps/<이름>.pgm/.yaml, 같은 이름 덮어쓰기).
- DURATION 초가 지나면 / Ctrl+C 하면 → 멈추고 마지막으로 지도 저장.

전제: robot.launch.py + slam_toolbox 가 이미 떠 있음.
정확한 탐색(빈 곳 찾아가기)이 아니라 '부딪히지 않고 헤매기'. 정식은 Nav2 + explore (내일 이후).

실행 (Pi):
  python3 ~/semantic-companion-robot/scripts/wander.py --minutes 10
  라이다 0° 가 로봇 뒤쪽이면  --laser-yaw 3.1416
"""
import argparse
import math
import os
import subprocess
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import LaserScan

FWD_SPEED = 0.08      # m/s  전진 (느리게: 라이다 10 Hz → 한 스캔에 0.8 cm 이동)
TURN_SPEED = 0.4      # rad/s 제자리 회전 (한 바퀴 ~16 s)
STOP_DIST = 0.40      # m  앞(±30°)에 이보다 가까운 게 있으면 회전 시작
CLEAR_DIST = 0.65     # m  회전하다 앞이 이만큼 비면 다시 전진
SIDE_DIST = 0.22      # m  옆(±30~80°)이 이보다 가까우면 그쪽에서 살짝 비켜감
MIN_VALID = 0.12      # m  이보다 가까운 값은 자기 몸/노이즈로 보고 무시
SCAN_TIMEOUT = 0.5    # s  스캔이 이만큼 안 오면 정지


class Wander(Node):
    def __init__(self, laser_yaw, map_name, save_every):
        super().__init__("wander")
        self.laser_yaw = laser_yaw
        self.map_name = map_name
        self.save_every = save_every
        self.pub = self.create_publisher(TwistStamped, "/cmd_vel", 10)
        self.create_subscription(LaserScan, "/scan", self.on_scan, 10)
        self.scan = None
        self.scan_time = 0.0
        self.turning = 0          # 0 = 전진 모드, +1 왼쪽 회전, -1 오른쪽 회전
        self.last_save = time.time()
        self.create_timer(0.1, self.tick)   # 10 Hz 제어

    def on_scan(self, msg):
        self.scan = msg
        self.scan_time = time.time()

    def sector_min(self, a0, a1):
        """로봇 기준 각도 a0~a1 (rad, 0=앞, +=왼쪽) 안의 가장 가까운 거리."""
        s = self.scan
        best = float("inf")
        for i, r in enumerate(s.ranges):
            if not (MIN_VALID < r < s.range_max) or math.isinf(r) or math.isnan(r):
                continue
            a = s.angle_min + i * s.angle_increment + self.laser_yaw
            a = math.atan2(math.sin(a), math.cos(a))   # -pi..pi 로 정리
            if a0 <= a <= a1 and r < best:
                best = r
        return best

    def send(self, v, w):
        m = TwistStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = "base_link"
        m.twist.linear.x = v
        m.twist.angular.z = w
        self.pub.publish(m)

    def tick(self):
        if self.scan is None or time.time() - self.scan_time > SCAN_TIMEOUT:
            self.send(0.0, 0.0)
            return
        d30 = math.radians(30)
        d80 = math.radians(80)
        front = self.sector_min(-d30, d30)
        left = self.sector_min(d30, d80)
        right = self.sector_min(-d80, -d30)

        if self.turning:
            if front > CLEAR_DIST:
                self.turning = 0
            else:
                self.send(0.0, self.turning * TURN_SPEED)
        if not self.turning:
            if front < STOP_DIST:
                # 왼쪽(30~120°) vs 오른쪽 중 더 넓은 쪽으로 돈다
                wide_l = self.sector_min(d30, math.radians(120))
                wide_r = self.sector_min(-math.radians(120), -d30)
                self.turning = 1 if wide_l >= wide_r else -1
                self.get_logger().info(f"막힘 front={front:.2f} m → {'왼쪽' if self.turning > 0 else '오른쪽'} 회전")
                self.send(0.0, self.turning * TURN_SPEED)
            else:
                w = 0.0
                if left < SIDE_DIST:
                    w = -0.3
                elif right < SIDE_DIST:
                    w = 0.3
                self.send(FWD_SPEED, w)

        if time.time() - self.last_save > self.save_every:
            self.save_map()

    def save_map(self):
        self.last_save = time.time()
        path = os.path.expanduser(f"~/maps/{self.map_name}")
        os.makedirs(os.path.dirname(path), exist_ok=True)
        # 별도 프로세스로 (몇 초 걸림, 그동안 이 노드는 계속 돈다)
        subprocess.Popen(["ros2", "run", "nav2_map_server", "map_saver_cli", "-f", path],
                         stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        self.get_logger().info(f"지도 저장 → {path}.pgm")


def check(node):
    """로봇 바로 앞 30 cm 에 물건을 두고 실행 → 가장 가까운 점이 0° 근처면 OK, ±180° 근처면 --laser-yaw 3.1416."""
    node.destroy_timer(node._timers[0])        # 움직이는 제어는 끔
    t_end = time.time() + 5
    while time.time() < t_end:
        rclpy.spin_once(node, timeout_sec=0.1)
        s = node.scan
        if s is None:
            continue
        best, ang = float("inf"), 0.0
        for i, r in enumerate(s.ranges):
            if MIN_VALID < r < best and not math.isinf(r):
                best, ang = r, s.angle_min + i * s.angle_increment
        print(f"가장 가까운 물체: {best:.2f} m, 라이다 각도 {math.degrees(ang):+.0f}°")
        time.sleep(0.4)
    rclpy.try_shutdown()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--minutes", type=float, default=10.0)
    ap.add_argument("--laser-yaw", type=float, default=0.0, help="라이다 0°가 로봇 뒤면 3.1416")
    ap.add_argument("--name", default=time.strftime("room_%m%d_%H%M"))
    ap.add_argument("--save-every", type=float, default=60.0)
    ap.add_argument("--check", action="store_true", help="움직이지 않고 가장 가까운 물체의 각도만 출력 (라이다 방향 확인)")
    a = ap.parse_args()

    rclpy.init()
    node = Wander(a.laser_yaw, a.name, a.save_every)
    if a.check:
        check(node)
        return
    end = time.time() + a.minutes * 60
    node.get_logger().info(f"시작: {a.minutes:.0f}분, 지도 이름 {a.name}. 멈추려면 Ctrl+C")
    try:
        while rclpy.ok() and time.time() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    for _ in range(5):              # 확실히 멈추기
        node.send(0.0, 0.0)
        time.sleep(0.05)
    path = os.path.expanduser(f"~/maps/{a.name}")
    os.makedirs(os.path.dirname(path), exist_ok=True)
    print(f"멈춤. 마지막 지도 저장 중 → {path}.pgm")
    subprocess.run(["ros2", "run", "nav2_map_server", "map_saver_cli", "-f", path])
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
