#!/usr/bin/env python3
"""[Pi] 저속 주행 확인 (브리지 v5 PWM 모드). 로봇을 바닥에, 앞뒤로 50 cm 이상 비우고.

  터미널 A: ros2 launch scr_bringup robot.launch.py ugv_port:=/dev/ttyAMA0 drive_mode:=pwm use_lidar:=false
  터미널 B: python3 scripts/crawl_test.py
  각 동작 4초 → 1초 정지. 앞뒤·좌우를 짝지어서 제자리 근처로 돌아옴.
  보는 것: 가다 서다 없이 잔잔한가 / 출발 덜컹 / 회전이 끊기지 않는가. 브리지 창의 "pwm:" 줄 = 실제로 보낸 힘.
"""
import time
import rclpy
from geometry_msgs.msg import TwistStamped

STEPS = [  # (설명, x m/s, z rad/s)
    ("직진 0.05", 0.05, 0.0), ("후진 0.05", -0.05, 0.0),
    ("직진 0.08", 0.08, 0.0), ("후진 0.08", -0.08, 0.0),
    ("제자리 왼쪽 0.2", 0.0, 0.2), ("제자리 오른쪽 0.2", 0.0, -0.2),
    ("호 왼쪽 (0.06, 0.3)", 0.06, 0.3), ("호 오른쪽 (0.06, -0.3)", 0.06, -0.3),
    ("아주 느린 명령 0.02 → 바닥선으로", 0.02, 0.0), ("후진 0.02", -0.02, 0.0),
]


def main():
    rclpy.init()
    node = rclpy.create_node("crawl_test")
    pub = node.create_publisher(TwistStamped, "cmd_vel", 10)
    time.sleep(1.0)                                   # 브리지와 연결될 시간
    msg = TwistStamped()
    try:
        for name, x, z in STEPS:
            print(f"▶ {name}")
            t0 = time.time()
            while time.time() - t0 < 4.0:
                msg.header.stamp = node.get_clock().now().to_msg()
                msg.twist.linear.x, msg.twist.angular.z = x, z
                pub.publish(msg)
                time.sleep(0.05)                      # 20 Hz, Nav2 와 비슷
            msg.twist.linear.x = msg.twist.angular.z = 0.0
            pub.publish(msg)
            time.sleep(1.0)                           # 침묵 → 브리지 데드맨이 정지
    finally:
        msg.twist.linear.x = msg.twist.angular.z = 0.0
        for _ in range(5):
            pub.publish(msg); time.sleep(0.05)
        node.destroy_node(); rclpy.shutdown()
        print("끝")


if __name__ == "__main__":
    main()
