#!/usr/bin/env python3
"""face_perception — 보기 담당 노드 (11-12, P1 1단계: 검출 + 추적. 인식은 다음 단계).

  입력  camera/image_raw     영상 (640x480·10fps 권장. 검출은 det_w 폭으로 줄여서)
        pantilt/cmd          고개 명령 [팬, 틸트, …] — 영상 찍힐 때 고개가 어디였는지 기록용 (look 이 보냄)
  출력  people               scr_interfaces/People: 확인된 트랙 전부 (얼굴 큰 순)

  한 장 처리 순서
    1. 찍힌 시각 = 받은 시각 − latency. 그때 고개 각도를 명령 기록에서 찾음
       (고개 명령이 바뀐 직후 settle_s 동안 찍힌 영상은 버림 — 서보가 아직 도는 중이라 각도가 틀림)
    2. YuNet → 얼굴마다 상자·점 5개·점수
    3. 얼굴 방향 = 고개 + 화면 각도, 얼굴 돌아간 정도(yaw/pitch) = 점 5개로
    4. face_track.Tracker 로 이어 붙임 → 확인된 트랙을 /people 로
  로그  1 s 마다 PEOPLE [...]
"""
import collections
import os
import sys
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from scr_interfaces.msg import People, Person
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from face_track import Tracker, face_angles, yaw_pitch  # noqa: E402


class FacePerception(Node):
    def __init__(self):
        super().__init__("face_perception")
        p = self.declare_parameter
        self.det_w = p("det_w", 640).value                 # 검출 폭. 11-12 실측: 320 은 바닥에서 먼 얼굴 놓침, 640 은 잡음 (55 ms)
        self.lat = p("latency_s", 0.15).value              # 찍힘 → 받음
        self.settle = p("settle_s", 0.5).value             # 고개 명령 바뀐 뒤 이만큼은 영상 버림
        model = os.path.expanduser(p("face_model", "~/models/face_yunet.onnx").value)
        self.det = cv2.FaceDetectorYN.create(model, "", (self.det_w, self.det_w), 0.6)
        self.tracker = Tracker()
        self.head = collections.deque(maxlen=100)          # (받은 시각, 팬, 틸트)
        self.moved_t = 0.0                                 # 마지막으로 고개 명령이 바뀐 시각
        self.log_t, self.n_frames, self.n_skip, self.det_ms = 0.0, 0, 0, 0.0
        self.create_subscription(Image, "camera/image_raw", self.on_img, qos_profile_sensor_data)
        self.create_subscription(Float32MultiArray, "pantilt/cmd", self.on_head, 10)
        self.pub = self.create_publisher(People, "people", 10)
        self.get_logger().info(f"PEOPLE 준비: YuNet 검출 폭 {self.det_w}, 추적 (인식은 아직)")

    # ---------- 고개 기록 ----------
    def on_head(self, m):
        pan, tilt = float(m.data[0]), float(m.data[1])
        if not self.head or abs(pan - self.head[-1][1]) > 0.5 or abs(tilt - self.head[-1][2]) > 0.5:
            self.moved_t = time.time()
        self.head.append((time.time(), pan, tilt))

    def head_at(self, t):
        for ht, hp, htl in reversed(self.head):
            if ht <= t:
                return hp, htl
        return (self.head[0][1], self.head[0][2]) if self.head else (0.0, 10.0)   # 기록 없으면 look 의 시작 자세

    # ---------- 영상 한 장 ----------
    def on_img(self, m):
        now = time.time()
        shot = now - self.lat
        if shot < self.moved_t + self.settle:
            self.n_skip += 1                                # 고개 도는 중에 찍힌 영상
            return
        self.n_frames += 1
        a = np.frombuffer(bytes(m.data), np.uint8).reshape(m.height, m.step)[:, : m.width * 3].reshape(m.height, m.width, 3)
        bgr = a if m.encoding == "bgr8" else np.ascontiguousarray(a[:, :, ::-1])
        h, w = bgr.shape[:2]
        s = min(1.0, self.det_w / w)
        small = cv2.resize(bgr, (int(w * s), int(h * s))) if s < 1.0 else bgr
        self.det.setInputSize((small.shape[1], small.shape[0]))
        t0 = time.time()
        _, faces = self.det.detect(small)
        self.det_ms += (time.time() - t0) * 1000
        hp, ht = self.head_at(shot)
        dets = []
        for r in ([] if faces is None else faces):
            r = r.copy()
            r[:14] /= s                                     # 원본 좌표로
            x, y, fw, fh = r[:4]
            ex, ey = face_angles(x + fw / 2, y + fh / 2, w, h)
            yaw, pitch = yaw_pitch([(r[4 + 2 * i], r[5 + 2 * i]) for i in range(5)])
            dets.append({"pan": hp + ex, "tilt": ht + ey, "px": float(fw) * 320.0 / w, "det": float(r[14]),
                         "yaw": yaw, "pitch": pitch, "box": r[:4].tolist(), "lm": r[4:14].tolist()})
        self.tracker.step(dets, shot)
        self.publish(shot)

    def publish(self, shot):
        msg = People()
        msg.header.stamp = Time(seconds=shot).to_msg()
        msg.header.frame_id = "camera"
        for tr in self.tracker.confirmed():
            msg.people.append(Person(
                track_id=tr.id, name="", name_conf=0.0, votes=0,
                pan=float(tr.pan), tilt=float(tr.tilt), face_px=float(tr.px), det_score=float(tr.det),
                yaw=float(tr.yaw), pitch=float(tr.pitch),
                age_s=float(shot - tr.born), unseen_s=float(max(0.0, shot - tr.seen))))
        self.pub.publish(msg)
        if time.time() - self.log_t >= 1.0:
            self.log_t = time.time()
            ps = [f"#{p.track_id} 팬{p.pan:.0f} 틸트{p.tilt:.0f} {p.face_px:.0f}px yaw{p.yaw:.0f} pitch{p.pitch:.0f}"
                  f"{' (안보임 %.1fs)' % p.unseen_s if p.unseen_s > 0 else ''}" for p in msg.people]
            self.get_logger().info(f"PEOPLE {len(ps)}명 [{', '.join(ps)}]  처리 {self.n_frames}장 (검출 평균 {self.det_ms / max(1, self.n_frames):.0f} ms) / 고개 도는 중 버림 {self.n_skip}장")
            self.n_frames = self.n_skip = 0
            self.det_ms = 0.0


def main():
    rclpy.init()
    rclpy.spin(FacePerception())


if __name__ == "__main__":
    main()
