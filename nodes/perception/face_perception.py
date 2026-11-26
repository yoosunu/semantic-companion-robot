#!/usr/bin/env python3
"""face_perception — 보기 담당 노드 (11-12 검출 + 추적, 11-26 인식: 이 사람이 Jasper 인가).

  입력  camera/image_raw     영상 (640x480·10fps 권장. 검출은 det_w 폭으로 줄여서)
        pantilt/cmd          고개 명령 [팬, 틸트, …] — 영상 찍힐 때 고개가 어디였는지 기록용 (look 이 보냄)
        neri/profile         executive 설정표 — face_hz 칸 (0 = 영상 처리 멈춤: 잠. 보안·절약) (11-14 사령관 v1)
  출력  people               scr_interfaces/People: 확인된 트랙 전부 (얼굴 큰 순)
        people/bodies        String JSON [{id, name, pan, tilt, w, h, face, unseen}] — 몸 (11-26, 나중에 따라오기)

  한 장 처리 순서
    1. 찍힌 시각 = 받은 시각 − latency. 그때 고개 각도를 명령 기록에서 찾음
       (고개 명령이 바뀐 직후 settle_s 동안 찍힌 영상은 버림 — 서보가 아직 도는 중이라 각도가 틀림)
    2. YuNet → 얼굴마다 상자·점 5개·점수
    3. 얼굴 방향 = 고개 + 화면 각도, 얼굴 돌아간 정도(yaw/pitch) = 점 5개로
    4. face_track.Tracker 로 이어 붙임 → 확인된 트랙을 /people 로
    5. (11-26) 이번 장에 얼굴이 잡힌 확인된 트랙 중 rec_max 명까지 SFace → 등록 (~/.scr_faces/<owner>.npy) 과 유사도
       → 트랙마다 face_owner.OwnerVote → Person.name = "jasper" / "stranger" / "" (모름), name_conf = 이번 유사도, votes
       규칙 (11-26 실측 face_eval replay): 0.40 이상 3 장 중 2 장 = jasper (공개 사진 215 명 중 착각 0, 나는 2~3 장),
       잘 보인 장 (≥10 px, 검출 ≥0.80) 5 장 전부 <0.30 = stranger. 우선 순서: 모름 → 큰 얼굴
    6. (11-26 Jasper: 얼굴과 몸은 하나) person_hz (2 Hz) 로 YOLO26n person → 몸 추적 (person_bind) → 얼굴이 몸 윗부분 안이면 묶음.
       이름의 주인은 몸: 뒤돌아 얼굴이 사라져도 유지, 다시 보인 얼굴 (새 추적) 은 바로 물려받음. 몸 목록 → people/bodies (JSON)
       YOLO 모델이 없으면 몸 없이 얼굴 추적이 이름을 가짐 (5 까지만)
  로그  1 s 마다 PEOPLE [...]
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
from rclpy.time import Time
from scr_interfaces.msg import People, Person
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray, String

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from face_track import Tracker, face_angles, yaw_pitch  # noqa: E402
import face_owner  # noqa: E402
import person_bind  # noqa: E402
from face_track import HFOV, VFOV  # noqa: E402

cv2.setNumThreads(2)   # 11-13 cpu_bench: 기본(4코어 전부)이면 YuNet 640 하나가 코어 1.4개 → 다른 노드와 뺏음


class FacePerception(Node):
    def __init__(self):
        super().__init__("face_perception")
        p = self.declare_parameter
        self.det_w = p("det_w", 640).value                 # 검출 폭. 11-12 실측: 320 은 바닥에서 먼 얼굴 놓침, 640 은 잡음 (55 ms)
        self.lat = p("latency_s", 0.15).value              # 찍힘 → 받음
        self.settle = p("settle_s", 0.5).value             # 고개 명령 바뀐 뒤 이만큼은 영상 버림
        self.rate = p("rate_hz", 5.0).value                 # 검출 주기 (11-13: 영상마다 10 Hz → 5 Hz, 도약+응시엔 충분)
        self.last_run = 0.0
        model = os.path.expanduser(p("face_model", "~/models/face_yunet.onnx").value)
        self.det = cv2.FaceDetectorYN.create(model, "", (self.det_w, self.det_w), 0.6)
        self.tracker = Tracker()
        # 11-26 인식
        self.owner = p("owner", "jasper").value
        self.rec_max = p("rec_max", 2).value                # 한 장에 인식할 최대 얼굴 수 (CPU 예산)
        self.rec_min_px = p("rec_min_px", 8.0).value         # 이보다 작은 얼굴은 인식 안 함 (320 기준)
        self.vote_kw = dict(thr=p("rec_thr", 0.40).value, k=2, n=3, not_thr=p("not_thr", 0.30).value, not_k=5)
        self.rec, self.gal = None, None
        gpath = os.path.join(os.path.expanduser("~/.scr_faces"), self.owner + ".npy")
        if os.path.exists(gpath):
            g = np.load(gpath).astype(np.float32)
            self.gal = g / np.linalg.norm(g, axis=1, keepdims=True)
            self.rec = cv2.FaceRecognizerSF.create(os.path.expanduser(p("rec_model", "~/models/face_sface.onnx").value), "")
        self.binder = person_bind.PeopleBinder(lambda: face_owner.OwnerVote(**self.vote_kw))   # 얼굴·몸 묶기 + 이름 (투표) 주인
        self.rec_ms, self.n_rec = 0.0, 0
        # 11-26 몸 (YOLO26n person)
        self.person_hz = p("person_hz", 2.0).value
        self.person_conf = p("person_conf", 0.4).value
        self.yolo, self.last_person, self.yolo_ms, self.n_yolo = None, 0.0, 0.0, 0
        ypath = os.path.expanduser(p("person_model", "~/models/yolo26n.onnx").value)
        if self.person_hz > 0 and os.path.exists(ypath):
            from yolo_onnx import Yolo
            self.yolo = Yolo(ypath, size=320, conf=self.person_conf)
        self.pub_bodies = self.create_publisher(String, "people/bodies", 10)
        self.head = collections.deque(maxlen=100)          # (받은 시각, 팬, 틸트)
        self.moved_t = 0.0                                 # 마지막으로 고개 명령이 바뀐 시각
        self.log_t, self.n_frames, self.n_skip, self.det_ms = 0.0, 0, 0, 0.0
        self.create_subscription(Image, "camera/image_raw", self.on_img, qos_profile_sensor_data)
        self.create_subscription(Float32MultiArray, "pantilt/cmd", self.on_head, 10)
        self.create_subscription(String, "neri/profile", self.on_profile, 10)
        self.enabled = True
        self.pub = self.create_publisher(People, "people", 10)
        self.get_logger().info(f"PEOPLE 준비: YuNet 검출 폭 {self.det_w}, 추적, 인식 "
                               + (f"{self.owner} 등록 {len(self.gal)} 장, 문턱 {self.vote_kw['thr']} 2/3, 아님 <{self.vote_kw['not_thr']}×5, 장마다 최대 {self.rec_max} 명"
                                  if self.gal is not None else f"끔 (등록 {gpath} 없음)")
                               + (f" | 몸: YOLO26n person {self.person_hz:.0f} Hz ≥{self.person_conf}" if self.yolo else f" | 몸: 끔 ({ypath} 없음 또는 person_hz 0)"))

    def on_profile(self, m):
        try:
            hz = float(json.loads(m.data).get("face_hz", self.rate))
        except ValueError:
            return
        if (hz > 0) != self.enabled or (hz > 0 and hz != self.rate):
            self.get_logger().info(f"PEOPLE 설정표: 얼굴 {'%.0f Hz' % hz if hz > 0 else '끔'}")
        self.enabled = hz > 0
        if hz > 0:
            self.rate = hz

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
        if not self.enabled:
            return
        if time.time() - self.last_run < 1.0 / self.rate:
            return                                          # 주기 전 영상은 건너뜀
        self.last_run = time.time()
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
        self.binder.drop_faces({tr.id for tr in self.tracker.tracks.values()})
        if self.yolo is not None and now - self.last_person >= 1.0 / self.person_hz:
            self.last_person = now
            self.bodies(bgr, shot, hp, ht)
        self.recognize(bgr, shot)
        self.publish(shot)

    def bodies(self, bgr, shot, hp, ht):
        """11-26 YOLO26n person → 몸 (각도) → 몸 추적 → 이번 장에 보인 얼굴을 몸 윗부분에 묶음"""
        h, w = bgr.shape[:2]
        t0 = time.time()
        found = [d for d in self.yolo.detect(bgr) if d[0] == "person"]
        self.yolo_ms += (time.time() - t0) * 1000
        self.n_yolo += 1
        dets = []
        for _, conf, x, y, bw, bh in found:
            ex, ey = face_angles(x + bw / 2, y + bh / 2, w, h)
            dets.append({"pan": float(hp + ex), "tilt": float(ht + ey), "w": float(bw * HFOV / w), "h": float(bh * VFOV / h)})
        self.binder.step_bodies(dets, shot)
        faces = [(tr.id, tr.pan, tr.tilt) for tr in self.tracker.tracks.values() if tr.seen == shot]
        self.binder.bind(faces, shot)
        self.pub_bodies.publish(String(data=json.dumps(self.binder.body_list(shot), ensure_ascii=False)))

    def recognize(self, bgr, shot):
        """11-26 이번 장에 잡힌 확인된 트랙 → SFace → OwnerVote (모름 먼저, 큰 얼굴 먼저, rec_max 명까지)"""
        if self.rec is None:
            return
        now = [tr for tr in self.tracker.confirmed() if tr.seen == shot and tr.box is not None and tr.px >= self.rec_min_px]
        now.sort(key=lambda tr: (self.binder.name_of(tr.id) != "", -tr.px))     # 모름 먼저, 큰 얼굴 먼저
        for tr in now[: self.rec_max]:
            row = np.array(list(tr.box) + list(tr.lm) + [tr.det], np.float32)
            t0 = time.time()
            feat = self.rec.feature(self.rec.alignCrop(bgr, row))
            self.rec_ms += (time.time() - t0) * 1000
            self.n_rec += 1
            f = feat[0] / np.linalg.norm(feat[0])
            cos = float((self.gal @ f).max())
            v = self.binder.vote_of(tr.id)                   # 몸에 묶였으면 몸의 투표, 아니면 얼굴 자기 것
            st, changed = v.update(cos, tr.px, tr.det)
            tr.cos = cos
            if changed:
                bid = self.binder.face_body.get(tr.id)
                self.get_logger().info(f"PEOPLE #{tr.id}{f' (몸 {bid})' if bid else ''} → {st or '모름'} "
                                       f"(유사도 {cos:.2f}, {tr.px:.0f}px, 검출 {tr.det:.2f}, {v.frames} 장째)")

    def publish(self, shot):
        msg = People()
        msg.header.stamp = Time(seconds=shot).to_msg()
        msg.header.frame_id = "camera"
        for tr in self.tracker.confirmed():
            msg.people.append(Person(
                track_id=tr.id, name=self.binder.name_of(tr.id), name_conf=float(getattr(tr, "cos", 0.0)), votes=0,
                pan=float(tr.pan), tilt=float(tr.tilt), face_px=float(tr.px), det_score=float(tr.det),
                yaw=float(tr.yaw), pitch=float(tr.pitch),
                age_s=float(shot - tr.born), unseen_s=float(max(0.0, shot - tr.seen))))
        self.pub.publish(msg)
        if time.time() - self.log_t >= 1.0:
            self.log_t = time.time()
            ps = [f"#{p.track_id} {p.name or '?'}({p.name_conf:.2f}) 팬{p.pan:.0f} 틸트{p.tilt:.0f} {p.face_px:.0f}px yaw{p.yaw:.0f} pitch{p.pitch:.0f}"
                  f"{' (안보임 %.1fs)' % p.unseen_s if p.unseen_s > 0 else ''}" for p in msg.people]
            bs = [f"몸{b['id']} {b['name'] or '?'}{' 얼굴#%d' % b['face'] if b['face'] else ' 얼굴없음'}{' (안보임 %.1fs)' % b['unseen'] if b['unseen'] > 0.6 else ''}"
                  for b in self.binder.body_list(shot)]
            self.get_logger().info(f"PEOPLE {len(ps)}명 [{', '.join(ps)}] 몸 [{', '.join(bs)}]  처리 {self.n_frames}장 (검출 평균 {self.det_ms / max(1, self.n_frames):.0f} ms"
                                   f", 인식 {self.n_rec}번 {self.rec_ms / max(1, self.n_rec):.0f} ms, YOLO {self.n_yolo}번 {self.yolo_ms / max(1, self.n_yolo):.0f} ms) / 고개 도는 중 버림 {self.n_skip}장")
            self.n_frames = self.n_skip = self.n_rec = self.n_yolo = 0
            self.det_ms = self.rec_ms = self.yolo_ms = 0.0


def main():
    rclpy.init()
    rclpy.spin(FacePerception())


if __name__ == "__main__":
    main()
