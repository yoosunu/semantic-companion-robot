#!/usr/bin/env python3
"""executive — 사령관 노드 v1 (11-14). 지금 모드를 정하고 설정표(/neri/profile)를 내려보냄.

  판단은 전부 executive_brain.Executive (ROS 없음, tests/executive_sim.py). 여기는 연결만.

  입력  neri/event     String JSON {"type": contact|approach|arrive|glance, "id": 트랙}  ← look 이 트리거 때
        people         scr_interfaces/People ← face_perception (얼굴이 보이나만 씀)
        scan           LiDAR — SLEEP·DOZE 일 때 "비어 있던 곳에 새로 들어옴" (lidar_motion) → 자극 (빔 20 개↑ 중, 아니면 약)
        imu            IMU — SLEEP·DOZE 일 때 톡톡 (imu_touch) → 강한 자극. 고개 명령 뒤 1 s·바퀴 움직임 중엔 안 셈
        pantilt/cmd, odom   자기가 만든 흔들림 빼기용 (efference copy)
        neri/request   String "sleep" | "wake" | "task:이름" | "done:이름"   (사람이 시험할 때:
                       ros2 topic pub --once /neri/request std_msgs/msg/String "{data: sleep}")
  출력  neri/profile   String JSON {"mode","why","face_hz","yolo_hz","floor_hz","head","wheels","nods","wake","stir"} — 바뀔 때 + 1 s 마다
                 방향은 전부 팬 각도 (°, + = 오른쪽 = /pantilt/cmd 와 같음). LiDAR·보드는 + = 왼쪽이라 부호 뒤집음
  로그  MODE 이전 → 다음 (이유) / 30 s 마다 지금 모드
"""
import json
import os
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from scr_interfaces.msg import People
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Float32MultiArray, String

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, "..", "perception"))
import executive_brain  # noqa: E402
from imu_touch import Touch  # noqa: E402
from lidar_motion import Motion  # noqa: E402

import math  # noqa: E402


class ExecutiveNode(Node):
    def __init__(self):
        super().__init__("executive")
        for k, (lo, hi) in executive_brain.DUR.items():     # 시간은 파라미터로 (최소, 최대): -p doze_after:="[20.0, 20.0]"
            v = list(self.declare_parameter(k, [float(lo), float(hi)]).value)
            executive_brain.DUR[k] = (float(v[0]), float(v[-1]))
        self.ex = executive_brain.Executive(time.time())
        self.touch = Touch()
        self.stimuli = []
        self.head_t, self.wheel_t = 0.0, 0.0
        self.events, self.requests = [], []
        self.people = False
        self.motion = None
        self.last_json, self.pub_t, self.log_t, self.n_log = "", 0.0, 0.0, 0
        self.create_subscription(String, "neri/event", self.on_event, 20)
        self.create_subscription(String, "neri/request", lambda m: self.requests.append(m.data.strip()), 10)
        self.create_subscription(People, "people", self.on_people, 10)
        self.create_subscription(LaserScan, "scan", self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Imu, "imu", self.on_imu, 20)
        self.create_subscription(Float32MultiArray, "pantilt/cmd", lambda m: setattr(self, "head_t", time.time()), 10)
        self.create_subscription(Odometry, "odom", self.on_odom, 10)
        self.pub = self.create_publisher(String, "neri/profile", 10)
        self.create_timer(0.1, self.tick)
        self.get_logger().info("EXEC 준비: " + ", ".join(f"{k} {a:.0f}~{b:.0f}s" for k, (a, b) in executive_brain.DUR.items()))

    def on_event(self, m):
        try:
            d = json.loads(m.data)
            self.events.append((d.get("type", ""), d.get("id", -1)))
        except ValueError:
            pass

    def on_people(self, m):
        self.people = any(p.unseen_s <= 0.5 for p in m.people)

    def on_odom(self, m):
        if abs(m.twist.twist.linear.x) > 0.02 or abs(m.twist.twist.angular.z) > 0.05:
            self.wheel_t = time.time()

    def on_scan(self, m):
        if self.ex.mode not in ("SLEEP", "DOZE"):
            self.motion = None
            return
        if self.motion is None:
            self.motion = Motion()                          # 졸기 시작할 때 3 s 동안 주변을 배움
        if self.motion.step(time.time(), list(m.ranges)):
            ang = m.angle_min + self.motion.last_idx * m.angle_increment     # LiDAR 각 (+ = 왼쪽)
            pan = -math.degrees(math.atan2(math.sin(ang), math.cos(ang)))  # → 팬 (+ = 오른쪽)
            s = 2 if self.motion.last >= 20 else 1
            self.stimuli.append(("lidar", s, round(pan, 1)))
            self.get_logger().info(f"STIM LiDAR: 빔 {self.motion.last} 개 들어옴, 팬 {pan:.0f}° 쪽, 세기 {s}")

    def on_imu(self, m):
        now = time.time()
        quiet = now - self.head_t > 1.0 and now - self.wheel_t > 1.0
        a, g = m.linear_acceleration, m.angular_velocity
        r = self.touch.step(now, a.x, a.y, g.x, g.y, quiet)
        if r and self.ex.mode in ("SLEEP", "DOZE"):
            d, size = r
            pan = None if d is None else round(-d, 1)
            self.stimuli.append(("touch", 3, pan))
            self.get_logger().info(f"STIM 톡톡: {size:.3f} g, 팬 {pan}° 쪽")

    def tick(self):
        now = time.time()
        n0 = len(self.ex.log)
        prof = self.ex.tick(now, self.events, self.people, self.stimuli, self.requests)
        self.events, self.requests, self.stimuli = [], [], []
        if self.ex.mode == "SLEEP":
            self.people = False                              # 얼굴 처리가 꺼지면 /people 이 안 옴 → 남은 값 버림
        for t, a, b, why in self.ex.log[n0:]:
            self.get_logger().info(f"MODE {a} → {b} ({why})" if a != b else f"MODE {a} ({why})")
        js = json.dumps(prof, ensure_ascii=False)
        if js != self.last_json or now - self.pub_t >= 1.0:
            self.last_json, self.pub_t = js, now
            self.pub.publish(String(data=js))
        if now - self.log_t >= 30.0:
            self.log_t = now
            self.get_logger().info(f"EXEC {prof['mode']} ({prof['why']}) {now - self.ex.since:.0f} s째")


def main():
    rclpy.init()
    rclpy.spin(ExecutiveNode())


if __name__ == "__main__":
    main()
