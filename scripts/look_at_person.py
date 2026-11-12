#!/usr/bin/env python3
"""look_at_person — 고개 담당 노드 v2 (11-12). 보지는 않고 /people 만 받아서 움직임.

  v1 (같은 날 아침): 이 노드가 직접 YuNet·YOLO 를 돌림 → 지금은 face_perception 이 봄 (640, 추적). 여기선 판단·움직임만.

  입력  people             scr_interfaces/People (face_perception)
        scan               LiDAR — 잠잘 때 가까운 움직임으로 깨기
  출력  pantilt/cmd        [팬, 틸트, SPD, ACC] → 브리지 v8.5
        pantilt/led        LED 밝기 0~255 (깨어있음 40 / 꾸벅 8 / 잠 0 / 반짝 255)
        perception/enable  Bool — 잠들면 False (face_perception 이 영상 처리 멈춤)
        look/state         String JSON {"mode", "reason", "target", "pan", "tilt", "people"} (1 s 로그도 같은 내용)

  판단은 전부 look_brain.Brain (ROS 없음, tests/look_brain_sim.py 로 시험):
    scan 두리번 / attend 쳐다봄 / idle 딴짓 / doze 꾸벅 / sleep 잠.  쳐다볼 "이유"(트리거)가 있을 때만 쳐다봄.
  파라미터  doze_after (60 s), sleep_after (180 s), sleep (true — false 면 꾸벅까지만)
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
from std_msgs.msg import Bool, Float32, Float32MultiArray, String

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import look_brain  # noqa: E402


class Look(Node):
    def __init__(self):
        super().__init__("look_at_person")
        p = self.declare_parameter
        look_brain.DOZE_AFTER = float(p("doze_after", look_brain.DOZE_AFTER).value)
        look_brain.SLEEP_AFTER = float(p("sleep_after", look_brain.SLEEP_AFTER).value)
        if not p("sleep", True).value:
            look_brain.SLEEP_AFTER = 1e9
        self.brain = look_brain.Brain()
        self.motion = look_brain.Motion()
        self.people, self.people_t = [], 0.0
        self.moved = False
        self.log_t = 0.0
        self.create_subscription(People, "people", self.on_people, 10)
        self.create_subscription(LaserScan, "scan", self.on_scan, qos_profile_sensor_data)
        self.pub_cmd = self.create_publisher(Float32MultiArray, "pantilt/cmd", 10)
        self.pub_led = self.create_publisher(Float32, "pantilt/led", 10)
        self.pub_enable = self.create_publisher(Bool, "perception/enable", 10)
        self.pub_state = self.create_publisher(String, "look/state", 10)
        self.create_timer(0.1, self.tick)
        self.get_logger().info(f"LOOK v2 준비: /people 로 판단, 꾸벅 {look_brain.DOZE_AFTER:.0f} s, "
                               f"잠 {'끔' if look_brain.SLEEP_AFTER > 1e8 else '%.0f s' % look_brain.SLEEP_AFTER}")

    def on_people(self, m):
        self.people_t = time.time()
        self.people = [{"id": x.track_id, "pan": x.pan, "tilt": x.tilt, "px": x.face_px, "yaw": x.yaw,
                        "pitch": x.pitch, "unseen": x.unseen_s, "name": x.name} for x in m.people]

    def on_scan(self, m):
        if self.brain.mode == "sleep" and self.motion.step(time.time(), list(m.ranges)):
            self.moved = True

    def tick(self):
        now = time.time()
        people = self.people if now - self.people_t < 1.0 else []     # face_perception 이 멈췄으면 아무도 없음
        acts = self.brain.tick(now, people, self.moved)
        self.moved = False
        for a in acts:
            if a[0] == "move":
                self.pub_cmd.publish(Float32MultiArray(data=[float(a[1]), float(a[2]), float(a[3]), float(a[4])]))
            elif a[0] == "led":
                self.pub_led.publish(Float32(data=float(a[1])))
            elif a[0] == "perceive":
                self.pub_enable.publish(Bool(data=bool(a[1])))
                if a[1] is False:
                    self.motion = look_brain.Motion()                  # 잠들 때 LiDAR 기준 새로
        b = self.brain
        st = {"mode": b.mode, "reason": b.reason, "target": b.target, "pan": round(b.pan, 1), "tilt": round(b.tilt, 1),
              "people": [f"#{p['id']} yaw{p['yaw']:.0f} {p['px']:.0f}px" for p in people if p["unseen"] <= 0.5]}
        self.pub_state.publish(String(data=json.dumps(st, ensure_ascii=False)))
        if now - self.log_t > 1.0 or any(a[0] != "move" for a in acts):
            self.log_t = now
            self.get_logger().info("LOOK " + json.dumps(st, ensure_ascii=False))


def main():
    rclpy.init()
    rclpy.spin(Look())


if __name__ == "__main__":
    main()
