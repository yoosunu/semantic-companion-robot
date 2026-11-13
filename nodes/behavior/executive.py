#!/usr/bin/env python3
"""executive — 사령관 노드 v1 (11-14). 지금 모드를 정하고 설정표(/neri/profile)를 내려보냄.

  판단은 전부 executive_brain.Executive (ROS 없음, tests/executive_sim.py). 여기는 연결만.

  입력  neri/event     String JSON {"type": contact|approach|arrive|glance, "id": 트랙}  ← look 이 트리거 때
        people         scr_interfaces/People ← face_perception (얼굴이 보이나만 씀)
        scan           LiDAR — SLEEP 일 때만 "비어 있던 곳에 새로 들어옴" 확인 (lidar_motion)
        neri/request   String "sleep" | "wake" | "task:이름" | "done:이름"   (사람이 시험할 때:
                       ros2 topic pub --once /neri/request std_msgs/msg/String "{data: sleep}")
  출력  neri/profile   String JSON {"mode","why","face_hz","yolo_hz","floor_hz","head","wheels"} — 바뀔 때 + 1 s 마다
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
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, "..", "perception"))
import executive_brain  # noqa: E402
from lidar_motion import Motion  # noqa: E402


class ExecutiveNode(Node):
    def __init__(self):
        super().__init__("executive")
        for k in executive_brain.DUR:                       # 시간은 파라미터로 바꿔 시험 가능 (예: -p doze_after:=20.0)
            executive_brain.DUR[k] = float(self.declare_parameter(k, executive_brain.DUR[k]).value)
        self.ex = executive_brain.Executive(time.time())
        self.events, self.requests = [], []
        self.people = False
        self.motion, self.lidar_in = None, False
        self.last_json, self.pub_t, self.log_t, self.n_log = "", 0.0, 0.0, 0
        self.create_subscription(String, "neri/event", self.on_event, 20)
        self.create_subscription(String, "neri/request", lambda m: self.requests.append(m.data.strip()), 10)
        self.create_subscription(People, "people", self.on_people, 10)
        self.create_subscription(LaserScan, "scan", self.on_scan, qos_profile_sensor_data)
        self.pub = self.create_publisher(String, "neri/profile", 10)
        self.create_timer(0.1, self.tick)
        self.get_logger().info("EXEC 준비: " + ", ".join(f"{k} {v:.0f}s" for k, v in executive_brain.DUR.items()))

    def on_event(self, m):
        try:
            d = json.loads(m.data)
            self.events.append((d.get("type", ""), d.get("id", -1)))
        except ValueError:
            pass

    def on_people(self, m):
        self.people = any(p.unseen_s <= 0.5 for p in m.people)

    def on_scan(self, m):
        if self.ex.mode != "SLEEP":
            self.motion = None
            return
        if self.motion is None:
            self.motion = Motion()                          # 잠든 직후 3 s 동안 주변을 배움
        if self.motion.step(time.time(), list(m.ranges)):
            self.lidar_in = True
            self.get_logger().info(f"WAKE LiDAR: 비어 있던 1.5 m 안으로 빔 {self.motion.last} 개 들어옴")

    def tick(self):
        now = time.time()
        n0 = len(self.ex.log)
        prof = self.ex.tick(now, self.events, self.people, self.lidar_in, self.requests)
        self.events, self.requests, self.lidar_in = [], [], False
        if self.ex.mode == "SLEEP":
            self.people = False                              # 얼굴 처리가 꺼지면 /people 이 안 옴 → 남은 값 버림
        for t, a, b, why in self.ex.log[n0:]:
            self.get_logger().info(f"MODE {a} → {b} ({why})")
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
