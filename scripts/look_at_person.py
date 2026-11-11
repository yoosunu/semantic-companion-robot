#!/usr/bin/env python3
"""look_at_person — 사람을 찾아 얼굴을 쳐다보기 (11-12, NERI 첫 시선).

  입력  /camera/image_raw (camera_ros, bgr8/rgb8)
  출력  /pantilt/cmd  Float32MultiArray [팬°, 틸트°, SPD, ACC] → 브리지 v8.5 {"T":133} (팬 −180~180, 틸트 −30~90)
        /look/state   String JSON {"mode", "person", "face", "conf", "err", "pan", "tilt", "infer_ms"} (상태 머신·말하기용)

  보기  0.2 s 마다 최신 영상 → ① YuNet(얼굴)을 화면 전체(320 폭)에서 먼저 → 가장 큰 얼굴 가운데
        ② 얼굴이 없으면 YOLO11n(사람) 상자 위에서 15% (머리 쪽 — 멀거나 뒤돌았을 때)
  방향  영상이 찍힐 때(받은 시각 − 0.15 s) 고개가 어디였나(기록) + 화면 오차 = 사람 방향.

  움직임 (11-12 Jasper: "정밀 추적보다 확 돌려 맞추고, 위아래 훑는 생명체 느낌") — 눈의 "도약 + 응시":
    응시   얼굴이 화면 가운데 근처(좌우 ±10°, 위아래 ±8°)면 고개를 가만히 둠. 미세 추적 안 함 (덜덜 없음)
    도약   그 밖으로 나가 두 번 연속 보이면 → 한 번에 확 (서보 최고 속도) → 0.6 s 는 영상 무시 (도는 중 영상)
    인사   5 s 넘게 못 보던 얼굴이 나타나면 → 그쪽으로 도약 + 고개 끄덕 (틸트 −8° → 원래)
    훑기   사람을 2 s 놓치면 마지막 본 곳 응시 → 5 s 넘으면 두리번: 4~7 s 마다 좌우 한 곳 + 높이 살짝 바꿔 봄
  (11-12 1·2·3차의 부드러운 따라가기는 20 Hz 작은 걸음 = 서보가 걸음마다 섰다 갔다 → 덜덜. 가속 제한 10~40 도 안 됨)
  모델  ~/models/yolo11n.onnx (onnxruntime), ~/models/face_yunet.onnx (OpenCV ≥ 4.8 FaceDetectorYN, 없으면 얼굴 생략)
"""
import collections
import json
import os
import random
import time

import numpy as np
import onnxruntime as ort
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray, String

try:
    import cv2
except ImportError:
    cv2 = None

ZONE_X, ZONE_Y = 10.0, 8.0            # 응시: 이 안이면 가만히
SETTLE_S = 0.6                       # 도약 뒤 영상 무시
GREET_AFTER = 5.0                    # 이만큼 못 보던 얼굴 = 새로 만남 → 끄덕
SCAN_AFTER = 5.0                     # 이만큼 아무도 없으면 두리번
SCAN_PANS = (-70.0, -35.0, 0.0, 35.0, 70.0)


class Look(Node):
    def __init__(self):
        super().__init__("look_at_person")
        p = self.declare_parameter
        self.model = os.path.expanduser(p("model", "~/models/yolo11n.onnx").value)
        self.face_model = os.path.expanduser(p("face_model", "~/models/face_yunet.onnx").value)
        self.size = p("imgsz", 320).value
        self.conf = p("conf", 0.45).value
        self.hfov = p("hfov_deg", 120.0).value      # 160° 는 대각. 가로 ≈ 120° 로 시작 (화면 보며 조정)
        self.vfov = p("vfov_deg", 90.0).value
        self.lat = p("latency_s", 0.15).value       # 찍힘 → 받음
        self.pan, self.tilt = 0.0, 10.0              # 보낸 고개
        self.hist = collections.deque(maxlen=60)     # (t, pan, tilt) 보낸 고개 기록 (늦은 영상 보정)
        self.img, self.img_t = None, 0.0
        self.seen_t, self.face_t, self.log_t = 0.0, 0.0, 0.0
        self.busy_until = 0.0                        # 도약·끄덕 중 (영상 무시)
        self.out = None                              # 응시 구역 밖에서 한 번 본 방향 (두 번 연속이면 도약)
        self.queue = []                              # 예약된 고개 [(시각, 팬, 틸트)] — 끄덕
        self.scan_t = 0.0
        self.mode = "idle"
        self.sess = ort.InferenceSession(self.model, providers=["CPUExecutionProvider"])
        self.inp = self.sess.get_inputs()[0].name
        self.face = None
        if cv2 is not None and hasattr(cv2, "FaceDetectorYN") and os.path.exists(self.face_model):
            try:
                self.face = cv2.FaceDetectorYN.create(self.face_model, "", (160, 160), 0.5)   # 0.6 은 가까이서 자주 놓침
            except cv2.error as e:
                self.get_logger().warn(f"LOOK 얼굴 모델 못 씀 ({e}) → 머리 위치만")
        self.create_subscription(Image, "camera/image_raw", self.on_img, qos_profile_sensor_data)
        self.pub = self.create_publisher(Float32MultiArray, "pantilt/cmd", 10)
        self.state_pub = self.create_publisher(String, "look/state", 10)
        self.create_timer(0.2, self.tick)
        self.create_timer(0.05, self.play)
        self.move(0.0, 10.0)
        self.get_logger().info(f"LOOK 준비: {os.path.basename(self.model)} ({self.size}), "
                               f"얼굴={'YuNet' if self.face else '없음(머리 위치)'}, cv2={cv2.__version__ if cv2 else '없음'}")

    def on_img(self, m):
        self.img, self.img_t = m, time.time()

    def move(self, pan, tilt, settle=SETTLE_S):
        """한 번에 확 (서보 최고 속도). 도는 동안 영상은 무시"""
        if pan > 180.0:
            pan -= 360.0                                         # ±180 너머는 반대편 각도 (선 때문에 크게 돌아감)
        elif pan < -180.0:
            pan += 360.0
        self.pan, self.tilt = max(-180.0, min(180.0, pan)), max(-30.0, min(90.0, tilt))
        self.pub.publish(Float32MultiArray(data=[float(self.pan), float(self.tilt), 0.0, 0.0]))
        now = time.time()
        self.hist.append((now, self.pan, self.tilt))
        self.busy_until = now + settle

    def play(self):
        """예약된 고개 (끄덕) 차례대로"""
        if self.queue and time.time() >= self.queue[0][0]:
            _, pn, tl = self.queue.pop(0)
            self.move(pn, tl, settle=0.35)

    def pose_at(self, t):
        for ht, hp, htl in reversed(self.hist):
            if ht <= t:
                return hp, htl
        return (self.hist[0][1], self.hist[0][2]) if self.hist else (self.pan, self.tilt)

    def frame(self, m):
        a = np.frombuffer(bytes(m.data), np.uint8).reshape(m.height, m.step)[:, : m.width * 3].reshape(m.height, m.width, 3)
        return a[:, :, ::-1] if m.encoding == "bgr8" else a      # → RGB

    def detect(self, rgb):
        """가장 확실한 사람 (x, y, w, h, conf) — 원래 영상 좌표, 없으면 None"""
        h, w = rgb.shape[:2]
        s = self.size / max(h, w)
        nw, nh = int(round(w * s)), int(round(h * s))
        if cv2 is not None:
            small = cv2.resize(rgb, (nw, nh), interpolation=cv2.INTER_LINEAR)
        else:
            small = rgb[(np.arange(nh) / s).astype(int)][:, (np.arange(nw) / s).astype(int)]
        pad = np.full((self.size, self.size, 3), 114, np.uint8)
        px, py = (self.size - nw) // 2, (self.size - nh) // 2
        pad[py:py + nh, px:px + nw] = small
        x = pad.transpose(2, 0, 1)[None].astype(np.float32) / 255.0
        out = self.sess.run(None, {self.inp: x})[0][0]           # (84, N): cx, cy, w, h, 80 클래스
        score = out[4]
        i = int(np.argmax(score))
        if score[i] < self.conf:
            return None
        cx, cy, bw, bh = out[0, i], out[1, i], out[2, i], out[3, i]
        return (float((cx - px) / s), float((cy - py) / s), float(bw / s), float(bh / s), float(score[i]))   # numpy float32 → float (JSON)

    def find_face(self, rgb):
        """화면 전체(320 폭)에서 가장 큰 얼굴 가운데 (원래 영상 좌표, 정확도), 없으면 None"""
        if self.face is None:
            return None
        H, W = rgb.shape[:2]
        k = 320.0 / W
        small = cv2.resize(np.ascontiguousarray(rgb[:, :, ::-1]), (320, int(round(H * k))))   # YuNet 은 BGR
        self.face.setInputSize((small.shape[1], small.shape[0]))
        _, faces = self.face.detect(small)
        if faces is None or not len(faces):
            return None
        f = max(faces, key=lambda r: r[2] * r[3])
        return float((f[0] + f[2] / 2) / k), float((f[1] + f[3] / 2) / k), float(f[-1])

    def tick(self):
        now = time.time()
        m = self.img
        if m is None or now < self.busy_until or self.queue or self.img_t - self.lat < self.busy_until - SETTLE_S + 0.3:
            return                                               # 도는 중이거나, 돌기 전·도는 중에 찍힌 영상
        self.img = None
        shot = self.img_t - self.lat
        rgb = self.frame(m)
        t0 = time.time()
        fc = self.find_face(rgb)
        d = None if fc else self.detect(rgb)                     # 얼굴이 있으면 YOLO 는 안 돌림 (전력도 아낌)
        st = {"person": bool(fc or d), "face": fc is not None}
        if fc or d:
            if fc:
                hx, hy, c = fc
            else:
                x, y, bw, bh, c = d
                hx, hy = x, y - bh / 2 + 0.15 * bh               # 머리 쪽
            ex = (hx - m.width / 2) / (m.width / 2) * (self.hfov / 2)      # + = 오른쪽
            ey = (m.height / 2 - hy) / (m.height / 2) * (self.vfov / 2)    # + = 위
            bp, bt = self.pose_at(shot)
            wp, wt = bp + ex, bt + ey                            # 사람 방향
            st.update(conf=round(c, 2), err=[round(ex, 1), round(ey, 1)])
            new_face = fc and now - self.face_t > GREET_AFTER
            if fc:
                self.face_t = now
            self.seen_t = now
            if new_face:
                self.mode = "greet"                              # 오랜만의 얼굴: 확 보고 끄덕
                self.move(wp, wt)
                self.queue = [(now + 0.7, wp, wt - 8.0), (now + 1.05, wp, wt)]
                self.out = None
            elif abs(ex) <= ZONE_X and abs(ey) <= ZONE_Y:
                self.mode, self.out = "gaze", None               # 가운데 근처: 가만히 응시
            elif self.out and abs(self.out[0] - wp) < 15.0 and abs(self.out[1] - wt) < 15.0:
                self.mode, self.out = "saccade", None            # 두 번 연속 밖: 확 돌림
                self.move(wp, wt)
            else:
                self.out = (wp, wt)                              # 한 번은 지켜봄
        else:
            lost = now - self.seen_t
            if lost > SCAN_AFTER and now > self.scan_t:
                self.mode = "scan"                               # 두리번: 좌우 한 곳 + 높이 살짝
                choices = [x for x in SCAN_PANS if abs(x - self.pan) > 20.0]
                self.move(random.choice(choices), random.uniform(5.0, 25.0), settle=1.0)
                self.scan_t = now + random.uniform(4.0, 7.0)
            elif lost > 2.0 and self.mode != "scan":
                self.mode = "lost"
        st.update(mode=self.mode, pan=round(self.pan, 1), tilt=round(self.tilt, 1), infer_ms=round((time.time() - t0) * 1000))
        self.state_pub.publish(String(data=json.dumps(st)))
        if now - self.log_t > 1.0:
            self.log_t = now
            self.get_logger().info("LOOK " + json.dumps(st))


def main():
    rclpy.init()
    n = Look()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
