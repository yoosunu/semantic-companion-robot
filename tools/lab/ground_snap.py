#!/usr/bin/env python3
"""[Pi] 바닥 검증 데이터 남기기 (11-18) — 바닥에 정답 물건을 놓은 채로 고개 자세를 바꿔 가며 원본 사진 + 고개 명령을 저장.
   나중에 ground.py (렌즈·위치·0점) 를 고칠 때 물건을 다시 놓지 않고 이 사진들로 다시 맞춤.

  준비: 로봇 (브리지) + 카메라 640 480 켜기. floor_perception·ground_lab·look 은 끄기 (고개를 이 도구가 씀)
  쓰기: ground_snap.py [이름]        → ~/calib/ground_set/<이름>/pan+0_tilt-20.png ... + poses.json
  자세: 틸트 −30 −20 −10 0 +10 (팬 0) + 팬 −60 −30 +30 +60 (틸트 −20). 자세마다 고개 멈춘 뒤 1.5 s 기다리고 3 장 중 마지막
  정답은 같은 폴더 truth.txt 에 손으로 (차체 앞 끝 기준 앞 cm, 가운데 줄 기준 좌우 cm: 왼쪽 +)
"""
import json
import os
import sys
import time

import cv2
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray

POSES = [(0, -30), (0, -20), (0, -10), (0, 0), (0, 10), (-60, -20), (-30, -20), (30, -20), (60, -20)]


def main():
    name = sys.argv[1] if len(sys.argv) > 1 else time.strftime("%Y%m%d_%H%M")
    out = os.path.expanduser(f"~/calib/ground_set/{name}")
    os.makedirs(out, exist_ok=True)
    rclpy.init()
    n = rclpy.create_node("ground_snap")
    imgs = []
    n.create_subscription(Image, "camera/image_raw", lambda m: imgs.append(m), qos_profile_sensor_data)
    pub = n.create_publisher(Float32MultiArray, "pantilt/cmd", 10)
    t0 = time.time()
    while pub.get_subscription_count() == 0 and time.time() - t0 < 5:
        rclpy.spin_once(n, timeout_sec=0.1)
    if pub.get_subscription_count() == 0:
        print("pantilt/cmd 듣는 노드 없음 (브리지 꺼짐?) → 그만")
        return
    log = []
    for pan, tilt in POSES:
        for _ in range(5):
            pub.publish(Float32MultiArray(data=[float(pan), float(tilt), 400.0, 10.0]))
            rclpy.spin_once(n, timeout_sec=0.05)
        end = time.time() + 3.0                                  # 이동 + 멈춤
        while time.time() < end:
            rclpy.spin_once(n, timeout_sec=0.05)
        imgs.clear()
        while len(imgs) < 3:
            rclpy.spin_once(n, timeout_sec=0.2)
        m = imgs[-1]
        a = np.frombuffer(bytes(m.data), np.uint8).reshape(m.height, m.step)[:, : m.width * 3].reshape(m.height, m.width, 3)
        bgr = a if m.encoding == "bgr8" else np.ascontiguousarray(a[:, :, ::-1])
        f = f"pan{pan:+d}_tilt{tilt:+d}.png"
        cv2.imwrite(os.path.join(out, f), bgr)
        log.append({"file": f, "pan": pan, "tilt": tilt, "size": [m.width, m.height]})
        print(f"저장 {f}", flush=True)
    for _ in range(5):
        pub.publish(Float32MultiArray(data=[0.0, -20.0, 400.0, 10.0]))
        rclpy.spin_once(n, timeout_sec=0.05)
    json.dump({"poses": log, "date": time.strftime("%Y-%m-%d %H:%M"), "ground_py": "TILT_OFFSET 2.3, LASER_X 0.078"},
              open(os.path.join(out, "poses.json"), "w"), indent=1)
    print(f"끝: {out} ({len(log)} 장). 정답은 {out}/truth.txt 에")


if __name__ == "__main__":
    main()
