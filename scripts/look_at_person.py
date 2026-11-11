#!/usr/bin/env python3
"""look_at_person — 카메라로 사람을 찾아(YOLO, ONNX) 팬틸트로 쳐다보기 (11-12, NERI 첫 시선).

  입력  /camera/image_raw (camera_ros, bgr8/rgb8)
  출력  /pantilt/cmd  Float32MultiArray [팬°, 틸트°]  → 브리지 v8.4 가 {"T":133} 로 (팬 −180~180, 틸트 −30~90)
        /look/state   String JSON {"person": bool, "conf", "err": [x°, y°], "pan", "tilt"} (나중에 상태 머신·말하기용)

  방법  0.2 s 마다 최신 영상 한 장 → YOLO(사람만) → 가장 확실한 사람 상자의 "머리 쪽"(위에서 15%) →
        화면 가운데와의 각도 차이 (160° 어안 = 각도가 화면 거리에 비례한다고 봄) → 팬·틸트를 그 차이의 일부만큼 (gain) 움직임.
        고개가 돌면 카메라도 같이 도니까 매번 "지금 화면 기준" 차이만 보면 됨. 명령 직후 0.25 s 영상은 버림 (돌던 중 흐린 영상).
        3° 안이면 그대로 (떨지 않게). 사람이 5 s 안 보이면 틸트만 천천히 정면(10°)으로.
  모델  ~/models/yolo11n.onnx (입력 320, COCO 0 = person). 필요: onnxruntime (scripts/setup_yolo.sh)
"""
import json
import math
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
        self.gain = p("gain", 0.5).value
        self.dead = p("deadband_deg", 3.0).value
        self.pan, self.tilt = 0.0, 10.0
        self.img, self.img_t, self.cmd_t = None, 0.0, 0.0
        self.seen_t, self.log_t = 0.0, 0.0
        self.sess = ort.InferenceSession(self.model, providers=["CPUExecutionProvider"])
        self.inp = self.sess.get_inputs()[0].name
        self.create_subscription(Image, "camera/image_raw", self.on_img, qos_profile_sensor_data)
        self.pub = self.create_publisher(Float32MultiArray, "pantilt/cmd", 10)
        self.state_pub = self.create_publisher(String, "look/state", 10)
        self.create_timer(0.2, self.tick)
        self.send()
        self.get_logger().info(f"LOOK 준비: {self.model} ({self.size}), cv2={'있음' if cv2 else '없음(numpy)'}")

    def on_img(self, m):
        self.img, self.img_t = m, time.time()

    def send(self):
        self.pan = max(-180.0, min(180.0, self.pan))
        self.tilt = max(-30.0, min(90.0, self.tilt))
        self.pub.publish(Float32MultiArray(data=[float(self.pan), float(self.tilt)]))
        self.cmd_t = time.time()

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
        if m is None or self.img_t - self.cmd_t < 0.25:
            return                                               # 고개 돌던 중 영상은 버림
        self.img = None
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
            moved = False
            if abs(ex) > self.dead:
                self.pan += self.gain * ex
                moved = True
            if abs(ey) > self.dead:
                self.tilt += self.gain * ey
                moved = True
            if moved:
                self.send()
        elif now - self.seen_t > 5.0 and abs(self.tilt - 10.0) > 1.0:
            self.tilt += math.copysign(min(2.0, abs(10.0 - self.tilt)), 10.0 - self.tilt)
            self.send()
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
