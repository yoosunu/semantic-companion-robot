#!/usr/bin/env python3
"""look_at_person — 카메라로 사람을 찾아(YOLO, ONNX) 팬틸트로 쳐다보기 (11-12, NERI 첫 시선).

  입력  /camera/image_raw (camera_ros, bgr8/rgb8)
  출력  /pantilt/cmd  Float32MultiArray [팬°, 틸트°]  → 브리지 v8.4 가 {"T":133} 로 (팬 −180~180, 틸트 −30~90)
        /look/state   String JSON {"person": bool, "conf", "err": [x°, y°], "pan", "tilt"} (나중에 상태 머신·말하기용)

  방법  0.2 s 마다 최신 영상 한 장 → YOLO(사람만) → 가장 확실한 사람 상자의 "머리 쪽"(위에서 15%) →
        화면 가운데와의 각도 차이 (160° 어안 = 각도가 화면 거리에 비례한다고 봄).
        11-12 첫 시험: 뚝뚝 끊기고 지나쳤다 돌아옴 → 두 가지로 고침
          ① 늦은 영상 보정: 영상은 찍힌 지 ~0.15 s 뒤에 옴. 그때 고개가 어디 있었나(기록) + 화면 오차 = 목표.
             "지금 고개 + 오차"로 하면 이미 돈 만큼을 또 돌아서 지나침
          ② 부드럽게: 목표는 탐지 때만 바꾸고, 고개는 20 Hz 로 목표를 향해 조금씩 (남은 거리의 25%, 최대 90°/s)
        2° 안이면 목표 그대로 (떨지 않게). 사람이 5 s 안 보이면 틸트만 천천히 정면(10°)으로.
  모델  ~/models/yolo11n.onnx (입력 320, COCO 0 = person). 필요: onnxruntime (scripts/setup_yolo.sh)
"""
import collections
import json
import os
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


class Look(Node):
    def __init__(self):
        super().__init__("look_at_person")
        p = self.declare_parameter
        self.model = os.path.expanduser(p("model", "~/models/yolo11n.onnx").value)
        self.size = p("imgsz", 320).value
        self.conf = p("conf", 0.45).value
        self.hfov = p("hfov_deg", 120.0).value      # 160° 는 대각. 가로 ≈ 120° 로 시작 (화면 보며 조정)
        self.vfov = p("vfov_deg", 90.0).value
        self.gain = p("gain", 0.8).value             # 늦은 영상 보정이 있으니 크게
        self.dead = p("deadband_deg", 2.0).value
        self.lat = p("latency_s", 0.15).value       # 찍힘 → 받음
        self.vmax = p("max_speed_dps", 90.0).value
        self.pan, self.tilt = 0.0, 10.0              # 지금 보낸 고개
        self.tp, self.tt = 0.0, 10.0                 # 목표
        self.hist = collections.deque(maxlen=60)     # (t, pan, tilt) 보낸 고개 기록 (늦은 영상 보정)
        self.img, self.img_t, self.cmd_t = None, 0.0, 0.0
        self.seen_t, self.log_t = 0.0, 0.0
        self.sess = ort.InferenceSession(self.model, providers=["CPUExecutionProvider"])
        self.inp = self.sess.get_inputs()[0].name
        self.create_subscription(Image, "camera/image_raw", self.on_img, qos_profile_sensor_data)
        self.pub = self.create_publisher(Float32MultiArray, "pantilt/cmd", 10)
        self.state_pub = self.create_publisher(String, "look/state", 10)
        self.create_timer(0.2, self.tick)
        self.create_timer(0.05, self.smooth)
        self.send()
        self.get_logger().info(f"LOOK 준비: {self.model} ({self.size}), cv2={'있음' if cv2 else '없음(numpy)'}")

    def on_img(self, m):
        self.img, self.img_t = m, time.time()

    def send(self):
        self.pan = max(-180.0, min(180.0, self.pan))
        self.tilt = max(-30.0, min(90.0, self.tilt))
        self.pub.publish(Float32MultiArray(data=[float(self.pan), float(self.tilt)]))
        self.cmd_t = time.time()
        self.hist.append((self.cmd_t, self.pan, self.tilt))

    def pose_at(self, t):
        """t 시각에 보냈던 고개 (영상이 찍힐 때 고개가 어디 있었나)"""
        for ht, hp, htl in reversed(self.hist):
            if ht <= t:
                return hp, htl
        return (self.hist[0][1], self.hist[0][2]) if self.hist else (self.pan, self.tilt)

    def smooth(self):
        """20 Hz: 고개를 목표 쪽으로 남은 거리의 25%씩 (최대 90°/s) — 뚝뚝 대신 부드럽게"""
        self.tp = max(-180.0, min(180.0, self.tp))
        self.tt = max(-30.0, min(90.0, self.tt))
        dp, dt_ = self.tp - self.pan, self.tt - self.tilt
        if abs(dp) < 0.3 and abs(dt_) < 0.3:
            return
        lim = self.vmax * 0.05
        self.pan += max(-lim, min(lim, 0.25 * dp if abs(dp) > 1.0 else dp))
        self.tilt += max(-lim, min(lim, 0.25 * dt_ if abs(dt_) > 1.0 else dt_))
        self.send()

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
        return ((cx - px) / s, (cy - py) / s, bw / s, bh / s, float(score[i]))

    def tick(self):
        now = time.time()
        m = self.img
        if m is None:
            return
        self.img = None
        shot = self.img_t - self.lat                             # 이 영상이 찍힌 때
        rgb = self.frame(m)
        t0 = time.time()
        d = self.detect(rgb)
        dt = time.time() - t0
        st = {"person": d is not None, "pan": round(self.pan, 1), "tilt": round(self.tilt, 1), "infer_ms": round(dt * 1000)}
        if d:
            x, y, bw, bh, c = d
            hx, hy = x, y - bh / 2 + 0.15 * bh                   # 머리 쪽
            ex = (hx - m.width / 2) / (m.width / 2) * (self.hfov / 2)      # + = 오른쪽
            ey = (m.height / 2 - hy) / (m.height / 2) * (self.vfov / 2)    # + = 위
            st.update(conf=round(c, 2), err=[round(ex, 1), round(ey, 1)])
            self.seen_t = now
            bp, bt = self.pose_at(shot)                          # 찍힐 때 고개 + 화면 오차 = 사람 방향
            if abs(ex) > self.dead:
                self.tp = bp + self.gain * ex
            if abs(ey) > self.dead:
                self.tt = bt + self.gain * ey
            st["target"] = [round(self.tp, 1), round(self.tt, 1)]
        elif now - self.seen_t > 5.0:
            self.tt = 10.0                                       # 사람이 없으면 정면 높이로 (천천히 — smooth 가 몰고 감)
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
