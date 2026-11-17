#!/usr/bin/env python3
"""floor_perception — 바닥 이상 감지 노드 (11-18). 판단은 floor_detect.py, 픽셀 → 바닥은 ground.py (둘 다 ROS 없음).

  입력  camera/image_raw   영상 640x480 (렌즈 보정과 같은 해상도여야 함: ~/calib/fisheye_640x480.json)
        pantilt/cmd        고개 명령 [팬, 틸트, …] — 바닥 좌표 계산 + 고개 바뀐 뒤 settle_s 동안은 안 셈 (자기 움직임)
        neri/profile       사령관 설정표 floor_hz (0 = 끔). 설정표가 없으면 rate_hz
  출력  floor/points       sensor_msgs/PointCloud2 (base_footprint, z = 0.02): 후보 덩어리의 바닥 닿은 아래 끝 점들
        floor/blobs        String JSON [{"near": m, "y": 평균 좌우 m, "area": px}] 가까운 순 — 사람·로그용
  아직 안 하는 것 (다음 단계): costmap 연결, 여러 장면 반복 확인 (그림자·햇빛 거르기), 로봇이 움직일 때
  시험: 고개를 다른 노드가 안 움직이면 -p tilt0:=-20.0 처럼 지금 자세를 알려줌 (명령을 받으면 그걸 씀)
  확인용: -p stream_port:=8085 → Mac 브라우저 http://<Pi IP>:8085 (분홍 = 다름, 하늘색 선 = 보는 바닥 1.5 m, 주황 = 후보 + 거리)
  로그 1 s 마다: FLOOR n 개, 가장 가까운 거리, 처리 ms
"""
import collections
import json
import os
import sys
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Float32MultiArray, Header, String

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import floor_detect  # noqa: E402
import ground  # noqa: E402

cv2.setNumThreads(1)


class FloorPerception(Node):
    def __init__(self):
        super().__init__("floor_perception")
        p = self.declare_parameter
        self.rate = p("rate_hz", 2.0).value
        self.lat = p("latency_s", 0.15).value
        self.settle = p("settle_s", 0.6).value
        port = p("stream_port", 0).value
        self.head0 = (p("pan0", 0.0).value, p("tilt0", 10.0).value)   # 고개 명령을 아직 못 받았을 때 지금 자세 (look 시작 자세 0,10)
        K, D, size = ground.load_calib(os.path.expanduser(p("calib", ground.CALIB).value))
        self.fd = floor_detect.FloorDetect(K, D, size, th=p("th", 4.0).value,
                                             max_range=p("max_range", 1.5).value, l_weight=p("l_weight", 1.0).value)
        self.size = size
        self.enabled = True
        self.head = collections.deque(maxlen=100)
        self.moved_t, self.last_run = 0.0, 0.0
        self.log_t, self.n, self.n_skip, self.ms, self.last_blobs = 0.0, 0, 0, 0.0, []
        self.push = None
        if port:
            sys.path.insert(0, os.path.join(HERE, "..", "..", "tools", "lab"))
            from face_lab import serve
            self.push = serve(port)
        self.create_subscription(Image, "camera/image_raw", self.on_img, qos_profile_sensor_data)
        self.create_subscription(Float32MultiArray, "pantilt/cmd", self.on_head, 10)
        self.create_subscription(String, "neri/profile", self.on_profile, 10)
        self.pub_pts = self.create_publisher(PointCloud2, "floor/points", 10)
        self.pub_blobs = self.create_publisher(String, "floor/blobs", 10)
        self.get_logger().info(f"FLOOR 준비: {self.rate:.0f} Hz, 보는 바닥 {self.fd.max_range} m, 보정 {size[0]}x{size[1]}"
                               + (f", 스트림 {port}" if port else ""))

    def on_profile(self, m):
        try:
            hz = float(json.loads(m.data).get("floor_hz", self.rate))
        except ValueError:
            return
        if (hz > 0) != self.enabled:
            self.get_logger().info(f"FLOOR 설정표: {'%.0f Hz' % hz if hz > 0 else '끔'}")
        self.enabled = hz > 0
        if hz > 0:
            self.rate = hz

    def on_head(self, m):
        pan, tilt = float(m.data[0]), float(m.data[1])
        if not self.head or abs(pan - self.head[-1][1]) > 0.5 or abs(tilt - self.head[-1][2]) > 0.5:
            self.moved_t = time.time()
        self.head.append((time.time(), pan, tilt))

    def head_at(self, t):
        for ht, hp, htl in reversed(self.head):
            if ht <= t:
                return hp, htl
        return (self.head[0][1], self.head[0][2]) if self.head else self.head0

    def on_img(self, m):
        now = time.time()
        if not self.enabled or now - self.last_run < 1.0 / self.rate:
            return
        self.last_run = now
        shot = now - self.lat
        if shot < self.moved_t + self.settle:
            self.n_skip += 1                                       # 고개 도는 중 → 각도가 틀림
            return
        if (m.width, m.height) != tuple(self.size):
            self.get_logger().warn(f"영상 {m.width}x{m.height} ≠ 렌즈 보정 {self.size[0]}x{self.size[1]} → 안 셈", throttle_duration_sec=10)
            return
        a = np.frombuffer(bytes(m.data), np.uint8).reshape(m.height, m.step)[:, : m.width * 3].reshape(m.height, m.width, 3)
        bgr = a if m.encoding == "bgr8" else np.ascontiguousarray(a[:, :, ::-1])
        pan, tilt = self.head_at(shot)
        t0 = time.time()
        blobs, odd, mask = self.fd.step(bgr, pan, tilt)
        ms = (time.time() - t0) * 1000
        self.n += 1
        self.ms += ms
        self.last_blobs = blobs
        hdr = Header(frame_id="base_footprint")
        hdr.stamp = m.header.stamp
        pts = [(float(x), float(y), 0.02) for b in blobs for x, y in b["pts"]]
        self.pub_pts.publish(point_cloud2.create_cloud_xyz32(hdr, pts))
        self.pub_blobs.publish(String(data=json.dumps(
            [{"near": round(b["near"], 3), "y": round(float(b["pts"][:, 1].mean()), 3), "area": b["area"]} for b in blobs])))
        if self.push:
            self.push(floor_detect.draw(bgr, self.fd, blobs, mask, odd, ms))
        if now - self.log_t >= 1.0:
            near = f"가장 가까움 {blobs[0]['near'] * 100:.0f} cm" if blobs else "없음"
            self.get_logger().info(f"FLOOR {len(blobs)} 개, {near} | 고개 ({pan:+.0f},{tilt:+.0f}) | "
                                   f"{self.ms / max(self.n, 1):.0f} ms, {self.n} 장, 고개 움직여 건너뜀 {self.n_skip}"
                                   + (" | 기준칸 이상: 안 배움" if odd else ""))
            self.log_t, self.n, self.n_skip, self.ms = now, 0, 0, 0.0


def main():
    rclpy.init()
    rclpy.spin(FloorPerception())


if __name__ == "__main__":
    main()
