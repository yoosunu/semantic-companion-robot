#!/usr/bin/env python3
"""look_at_person — 사람을 찾아 얼굴을 쳐다보기 (11-12, NERI 첫 시선).

  입력  /camera/image_raw (camera_ros, bgr8/rgb8)
  출력  /pantilt/cmd  Float32MultiArray [팬°, 틸트°]  → 브리지 v8.4 {"T":133} (팬 −180~180, 틸트 −30~90)
        /look/state   String JSON {"person", "face", "conf", "err", "pan", "tilt", "target", "infer_ms"} (상태 머신·말하기용)

  보기  0.2 s 마다 최신 영상 → ① YuNet(얼굴)을 화면 전체(320 폭으로 줄여)에서 먼저 → 있으면 가장 큰 얼굴 가운데
        ② 얼굴이 없으면 YOLO11n(사람) 상자 위에서 15% (머리 쪽 — 멀거나 뒤돌았을 때).
        11-12 3차: 가까이서 위를 보면 몸이 화면에 다 안 들어와 YOLO 가 깜빡임(사람 있음/없음 반복) + 얼굴↔머리 추정 점이
        번갈아 바뀌어 목표가 10° 넘게 왔다 갔다 → 덜덜. 가까울수록 얼굴이 확실하니 얼굴 먼저.
        화면 가운데와의 각도 차이 (160° 어안 = 각도가 화면 거리에 비례한다고 봄).
  목표  영상이 찍힐 때(받은 시각 − 0.15 s) 고개가 어디였나(기록) + 화면 오차 = 사람 방향 (11-12 1차: "지금 고개 + 오차"는
        이미 돈 만큼 또 돌아 지나침). 탐지가 조금씩 튀는 건 평균(새 값 30%)으로 거름.
        25° 넘게 튀는 값은 한 번은 무시, 다음에도 그 근처면 새 사람.
        팬 목표가 ±180 을 넘으면 반대편 각도로 (선 때문에 그 사이를 못 지나감 → 크게 돌아감).
  움직임 (11-12 2차, Jasper: "자연스럽게") 20 Hz 스프링: 출발은 천천히 가속, 도착은 부드럽게 멈춤 (임계 감쇠).
        4° 넘게 벗어나야 움직이기 시작, 1.5° 안이면 멈춤 (작은 떨림 없음). 30° 넘는 큰 차이는 더 빠르게 (휙 돌아보기).
  놓침  2 s 는 마지막 본 곳을 바라봄 → 틸트만 천천히 정면 높이(10°) → 10 s 넘으면 팬도 천천히 정면(0°).
  모델  ~/models/yolo11n.onnx (onnxruntime), ~/models/face_yunet.onnx (OpenCV ≥ 4.8 FaceDetectorYN, 없으면 얼굴 생략)
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

DT = 0.05
START_DEG, STOP_DEG = 4.0, 1.5       # 움직이기 시작 / 멈춤 (히스테리시스)
WN, WN_FAST, FAST_DEG = 4.0, 7.0, 30.0   # 스프링 고유 진동수 [rad/s] — 4 ≈ 1.2 s 에 도착, 큰 차이는 7 (11-12: 5 는 덜덜)
WN_SLOW = 1.5                        # 놓쳤을 때 정면으로 돌아오기
VMAX = 150.0                         # °/s
JUMP_DEG = 25.0                      # 탐지가 이만큼 튀면 한 번은 무시, 두 번 연속이면 새 사람
ALPHA = 0.3                          # 평균: 새 값 30%


class Axis:
    """한 축 스프링 (임계 감쇠) + 히스테리시스"""
    def __init__(self, pos, lo, hi):
        self.pos, self.v, self.target, self.lo, self.hi, self.moving = pos, 0.0, pos, lo, hi, False

    def step(self, wn_override=None):
        e = self.target - self.pos
        if not self.moving and abs(e) > START_DEG:
            self.moving = True
        if self.moving and abs(e) < STOP_DEG and abs(self.v) < 5.0:
            self.moving, self.v = False, 0.0
        if not self.moving:
            return False
        wn = wn_override or (WN_FAST if abs(e) > FAST_DEG else WN)
        self.v += (wn * wn * e - 2.0 * wn * self.v) * DT
        self.v = max(-VMAX, min(VMAX, self.v))
        self.pos = max(self.lo, min(self.hi, self.pos + self.v * DT))
        return True


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
        # 서보 자체 속도·가속 (0 = 최고, 단위는 펌웨어 기준 — 문서에 없음). 11-12: 최고 속도로 20 Hz 작은 걸음 = 덜덜
        #   → 서보가 부드럽게 따라오게 값을 줌. 돌리는 중에 바꿔 보기: ros2 param set /look_at_person servo_acc 20
        p("servo_spd", 0)
        p("servo_acc", 20)
        self.pan, self.tilt = Axis(0.0, -180.0, 180.0), Axis(10.0, -30.0, 90.0)
        self.hist = collections.deque(maxlen=60)     # (t, pan, tilt) 보낸 고개 기록 (늦은 영상 보정)
        self.img, self.img_t = None, 0.0
        self.seen_t, self.log_t = 0.0, 0.0
        self.odd = None                              # 크게 튄 값 (한 번 보류)
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
        self.create_timer(DT, self.smooth)
        self.send()
        self.get_logger().info(f"LOOK 준비: {os.path.basename(self.model)} ({self.size}), "
                               f"얼굴={'YuNet' if self.face else '없음(머리 위치)'}, cv2={cv2.__version__ if cv2 else '없음'}")

    def on_img(self, m):
        self.img, self.img_t = m, time.time()

    def send(self):
        spd = float(self.get_parameter("servo_spd").value)
        acc = float(self.get_parameter("servo_acc").value)
        self.pub.publish(Float32MultiArray(data=[float(self.pan.pos), float(self.tilt.pos), spd, acc]))
        self.hist.append((time.time(), self.pan.pos, self.tilt.pos))

    def pose_at(self, t):
        for ht, hp, htl in reversed(self.hist):
            if ht <= t:
                return hp, htl
        return (self.hist[0][1], self.hist[0][2]) if self.hist else (self.pan.pos, self.tilt.pos)

    def smooth(self):
        lost = time.time() - self.seen_t
        slow = WN_SLOW if lost > 2.0 else None
        a = self.pan.step(slow)
        b = self.tilt.step(slow)
        if a or b:
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
        if m is None:
            return
        self.img = None
        shot = self.img_t - self.lat                             # 이 영상이 찍힌 때
        rgb = self.frame(m)
        t0 = time.time()
        fc = self.find_face(rgb)
        d = None if fc else self.detect(rgb)                     # 얼굴이 있으면 YOLO 는 안 돌림 (전력도 아낌)
        st = {"person": bool(fc or d), "face": fc is not None, "pan": round(self.pan.pos, 1), "tilt": round(self.tilt.pos, 1)}
        if fc or d:
            if fc:
                hx, hy, c = fc
            else:
                x, y, bw, bh, c = d
                hx, hy = x, y - bh / 2 + 0.15 * bh               # 머리 쪽
            ex = (hx - m.width / 2) / (m.width / 2) * (self.hfov / 2)      # + = 오른쪽
            ey = (m.height / 2 - hy) / (m.height / 2) * (self.vfov / 2)    # + = 위
            bp, bt = self.pose_at(shot)                          # 찍힐 때 고개 + 화면 오차 = 사람 방향
            wp, wt = bp + ex, bt + ey
            if wp > 180.0:
                wp -= 360.0                                      # ±180 을 넘으면 반대편 각도 (선 때문에 크게 돌아감)
            elif wp < -180.0:
                wp += 360.0
            far = max(abs(wp - self.pan.target), abs(wt - self.tilt.target)) > JUMP_DEG
            if now - self.seen_t > 1.0:
                self.pan.target, self.tilt.target, self.odd = wp, wt, None          # 오랜만 = 바로
            elif far:
                if self.odd and abs(self.odd[0] - wp) < 10.0 and abs(self.odd[1] - wt) < 10.0:
                    self.pan.target, self.tilt.target, self.odd = wp, wt, None      # 두 번 연속 = 진짜 옮겨감
                else:
                    self.odd = (wp, wt)                                              # 한 번은 보류
            else:
                self.odd = None
                self.pan.target += ALPHA * (wp - self.pan.target)
                self.tilt.target += ALPHA * (wt - self.tilt.target)
            self.seen_t = now
            st.update(conf=round(c, 2), err=[round(ex, 1), round(ey, 1)])
        else:
            lost = now - self.seen_t
            if lost > 2.0:
                self.tilt.target = 10.0                          # 정면 높이로 (천천히 — smooth 가 느린 스프링으로)
            if lost > 10.0:
                self.pan.target = 0.0
        st["target"] = [round(self.pan.target, 1), round(self.tilt.target, 1)]
        st["infer_ms"] = round((time.time() - t0) * 1000)
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
