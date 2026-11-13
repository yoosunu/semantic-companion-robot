#!/usr/bin/env python3
"""look_at_person — 고개 담당 노드 v2 (11-12). 보지는 않고 /people 만 받아서 움직임.

  v1 (같은 날 아침): 이 노드가 직접 YuNet·YOLO 를 돌림 → 지금은 face_perception 이 봄 (640, 추적). 여기선 판단·움직임만.

  입력  people             scr_interfaces/People (face_perception)
        neri/profile       executive 설정표 — head 칸 (free / doze / sleep) 만 따름 (11-14 사령관 v1)
        odom               바퀴 — 로봇이 움직이는 중엔 "다가옴" 안 셈 (우리가 다가가도 얼굴은 커짐)
  출력  pantilt/cmd        [팬, 틸트, SPD, ACC] → 브리지 v8.5
        pantilt/led        LED 밝기 0~255 (깨어있음 40 / 꾸벅 8 / 잠 0 / 반짝 255)
        neri/event         String JSON {"type", "id"} — 트리거(눈맞춤·다가옴·인사·두리번) 때 executive 에 알림
        look/state         String JSON {"mode", "reason", "target", "err", "pan", "tilt", "people"} (1 s 로그도 같은 내용)
                           err = 쳐다보는 사람 방향 − 고개 (°). attend 에서 0 근처면 눈맞춤 중

  판단은 전부 look_brain.Brain (ROS 없음, tests/look_brain_sim.py 로 시험):
    scan 두리번 / attend 쳐다봄 / idle 딴짓 / doze 꾸벅 / sleep 잠.  쳐다볼 "이유"(트리거)가 있을 때만 쳐다봄.
    **언제** 졸고 자고 깨는지는 executive 가 정함 (11-14). 여기는 모양만. executive 가 없으면 졸지 않음.
  파라미터  led (false — 11-12 실측: 8 도 눈부신 조명이라 표현에 안 씀)
  로그      LOOK {...} 1 s 마다 + TRIG 트리거가 생길 때마다 (다가옴·눈맞춤이 실제로 잡혔는지 확인용)
"""
import json
import os
import sys
import time

import rclpy
from rclpy.node import Node
from scr_interfaces.msg import People
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, Float32MultiArray, String

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import look_brain  # noqa: E402


class Look(Node):
    def __init__(self):
        super().__init__("look_at_person")
        p = self.declare_parameter
        self.led = p("led", False).value
        self.brain = look_brain.Brain()
        self.head = "free"                                        # executive 설정표의 head 칸
        self.prof = {}                                            # 설정표 전체 (nods·wake·stir)
        self.people, self.people_t = [], 0.0
        self.log_t = 0.0
        self.create_subscription(People, "people", self.on_people, 10)
        self.create_subscription(String, "neri/profile", self.on_profile, 10)
        self.create_subscription(Odometry, "odom", self.on_odom, 10)
        self.moving_t = 0.0                                       # 마지막으로 바퀴가 움직인 시각
        self.pub_cmd = self.create_publisher(Float32MultiArray, "pantilt/cmd", 10)
        self.pub_led = self.create_publisher(Float32, "pantilt/led", 10)
        self.pub_event = self.create_publisher(String, "neri/event", 20)
        self.pub_state = self.create_publisher(String, "look/state", 10)
        self.create_timer(0.1, self.tick)
        self.led_off_n = 0 if self.led else 3                     # led 끔이면 처음 3 s 동안 1 s 마다 0 보냄
        self.create_timer(1.0, self.led_off)                      # (브리지·펌웨어는 마지막 밝기를 그대로 들고 있음 → 예전 실행의 40 이 남아 있었음)
        self.get_logger().info("LOOK v2 준비: /people 로 판단, 졸기·잠은 executive 설정표(head) 따름")

    def led_off(self):
        if self.led_off_n > 0:
            self.led_off_n -= 1
            self.pub_led.publish(Float32(data=0.0))

    def on_people(self, m):
        self.people_t = time.time()
        self.people = [{"id": x.track_id, "pan": x.pan, "tilt": x.tilt, "px": x.face_px, "yaw": x.yaw,
                        "pitch": x.pitch, "unseen": x.unseen_s, "name": x.name} for x in m.people]

    def on_odom(self, m):
        if abs(m.twist.twist.linear.x) > 0.02 or abs(m.twist.twist.angular.z) > 0.05:
            self.moving_t = time.time()

    def on_profile(self, m):
        try:
            self.prof = json.loads(m.data)
            self.head = self.prof.get("head", "free")
        except ValueError:
            pass

    def tick(self):
        now = time.time()
        people = self.people if now - self.people_t < 1.0 else []     # face_perception 이 멈췄으면 아무도 없음
        pr = self.prof
        acts = self.brain.set_head(self.head, now, pr.get("nods", 3), pr.get("wake"), pr.get("stir", 0)) \
            + self.brain.tick(now, people, now - self.moving_t < 1.0, now - self.people_t)
        for name, pid in self.brain.events:                       # 트리거 → executive 에 알림
            self.pub_event.publish(String(data=json.dumps({"type": name, "id": -1 if pid is None else pid})))
            self.get_logger().info(f"EVENT {name} #{pid} → executive")
        self.brain.events = []
        if self.brain.last_trig and self.brain.last_trig != getattr(self, "logged_trig", None):
            self.logged_trig = self.brain.last_trig
            t, name, pid = self.brain.last_trig
            p = next((x for x in people if x["id"] == pid), {})
            self.get_logger().info(f"TRIG {name} #{pid} (yaw {p.get('yaw', 0):.0f}, {p.get('px', 0):.0f}px) → {self.brain.mode}")
        for a in acts:
            if a[0] == "move":
                self.pub_cmd.publish(Float32MultiArray(data=[float(a[1]), float(a[2]), float(a[3]), float(a[4])]))
            elif a[0] == "led" and self.led:
                self.pub_led.publish(Float32(data=float(a[1])))
        b = self.brain
        tgt = next((p for p in people if p["id"] == b.target and p["unseen"] <= 0.5), None)
        err = [round(tgt["pan"] - b.pan, 1), round(tgt["tilt"] - b.tilt, 1)] if tgt else None   # 쳐다보는 사람과 고개 차이 (°)
        st = {"mode": b.mode, "reason": b.reason, "target": b.target, "err": err, "pan": round(b.pan, 1), "tilt": round(b.tilt, 1),
              "people": [f"#{p['id']} yaw{p['yaw']:.0f} {p['px']:.0f}px" for p in people if p["unseen"] <= 0.5]}
        self.pub_state.publish(String(data=json.dumps(st, ensure_ascii=False)))
        if now - self.log_t > 1.0:
            self.log_t = now
            self.get_logger().info("LOOK " + json.dumps(st, ensure_ascii=False))


def main():
    rclpy.init()
    rclpy.spin(Look())


if __name__ == "__main__":
    main()
