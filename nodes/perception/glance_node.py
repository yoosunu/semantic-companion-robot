#!/usr/bin/env python3
"""glance_node — "안경 쓰기" 노드 (11-18. 처음 이름 inspect.py 는 파이썬 기본 모듈 inspect 를 가려서 같은 폴더 노드가 다 죽음 → 바꿈). 바닥 후보가 반복되면 사령관에 힐끗 요청 → 고개가 가면 YOLOE 한 번 → 결과·시간 기록.

  판단은 glance.py (순수, tests/glance_sim.py). 흐름 (11-13 설계, 11-18 Jasper: 걸으면서 힐끗, 애매하면 "이게 뭐지?" 는 다음 단계)
    floor/blobs (floor_perception, 2 Hz) → Candidates (세상 좌표 같은 자리 2 번, 1.2 m 안, 본 곳 5 분 기억)
    → neri/request "glance:{id,pan,tilt}" (IDLE·TASK 일 때만, 힐끗 사이 min_gap)
    → neri/profile 이 GLANCE + 같은 id → pantilt/cmd 가 그 각도로 바뀐 뒤 settle_s 기다림 → 그다음 영상으로 YOLOE
    → 후보 근처 (바닥 0.3 m) 상자 중 가장 높은 점수 = 이름, 없으면 "?" → semantic/seen + neri/event glanced (사령관 복귀)
  입력  floor/blobs, neri/profile, pantilt/cmd, camera/image_raw, odom
  출력  neri/request, neri/event, semantic/seen (String JSON {"id","name","conf","wx","wy","dist","ms":{…}})
  시간 (로그 GLANCE): 요청 → 사령관 GLANCE → 고개 명령 → 영상 → YOLOE 끝 (11-18 예산 ~1.4 s)
  그림: ~/glance_log/<시각>_<이름>.jpg (상자 + 후보 점). 집 안 사진 → 공개 repo 에 안 올림
  파이썬: onnxruntime 필요 → ~/yolo_venv/bin/python
"""
import collections
import json
import math
import os
import sys
import time

import cv2
import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray, String

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import glance as gl  # noqa: E402
import ground  # noqa: E402
from yolo_onnx import Yolo  # noqa: E402

cv2.setNumThreads(1)
OUT = os.path.expanduser("~/glance_log")


class Inspect(Node):
    def __init__(self):
        super().__init__("glance")
        p = self.declare_parameter
        self.settle = p("settle_s", 0.6).value
        self.min_gap = p("min_gap_s", 4.0).value          # 힐끗 사이 최소
        self.match_r = p("match_m", 0.3).value            # YOLOE 상자 아래 끝이 후보에서 이만큼 안이면 그 물건
        model = os.path.expanduser(p("model", "~/models/yoloe-26s-seg.onnx").value)
        self.yolo = Yolo(model, size=p("size", 320).value, conf=p("conf", 0.08).value)
        self.K, self.D, self.size = ground.load_calib()
        self.cands = gl.Candidates(max_dist=p("max_dist", 1.2).value)
        self.mode, self.prof_glance = "IDLE", None
        self.pose = (0.0, 0.0, 0.0)
        self.head_cmd, self.head_t = None, 0.0
        self.frame, self.frame_t = None, 0.0
        self.job = None                                   # 진행 중 힐끗 {"id","cand","pan","tilt","t":{…}}
        self.next_id, self.last_glance = 1, 0.0
        os.makedirs(OUT, exist_ok=True)
        self.create_subscription(String, "floor/blobs", self.on_blobs, 10)
        self.create_subscription(String, "neri/profile", self.on_profile, 10)
        self.create_subscription(Float32MultiArray, "pantilt/cmd", self.on_head, 10)
        self.create_subscription(Image, "camera/image_raw", self.on_img, qos_profile_sensor_data)
        self.create_subscription(Odometry, "odom", self.on_odom, 20)
        self.pub_req = self.create_publisher(String, "neri/request", 10)
        self.pub_ev = self.create_publisher(String, "neri/event", 10)
        self.pub_seen = self.create_publisher(String, "semantic/seen", 10)
        self.create_timer(0.05, self.tick)
        self.get_logger().info(f"GLANCE 준비: {os.path.basename(model)} 단어 {len(self.yolo.names)}개 "
                               f"({', '.join(self.yolo.names[:12])}{'…' if len(self.yolo.names) > 12 else ''})")

    # ---------- 입력 ----------
    def on_odom(self, m):
        q = m.pose.pose.orientation
        self.pose = (m.pose.pose.position.x, m.pose.pose.position.y,
                     math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))

    def on_profile(self, m):
        try:
            d = json.loads(m.data)
        except ValueError:
            return
        self.mode, self.prof_glance = d.get("mode", "IDLE"), d.get("glance")

    def on_head(self, m):
        cmd = (round(float(m.data[0]), 1), round(float(m.data[1]), 1))
        if cmd != self.head_cmd:
            self.head_cmd, self.head_t = cmd, time.time()

    def on_img(self, m):
        self.frame, self.frame_t = m, time.time()

    def on_blobs(self, m):
        if self.job or not (self.mode == "IDLE" or self.mode.startswith("TASK:")):
            return
        try:
            d = json.loads(m.data)
        except ValueError:
            return
        now = time.time()
        cand = self.cands.step(now, tuple(d["pose"]), [(b["x"], b["y"], b["area"]) for b in d["blobs"]])
        if not cand or now - self.last_glance < self.min_gap:
            return
        x, y = gl.to_robot(self.pose, cand["wx"], cand["wy"])          # 지금 위치 기준으로 다시 (걷는 중이면 조금 바뀜)
        pan, tilt = gl.head_for(x, y)
        gid, self.next_id = self.next_id, self.next_id + 1
        self.job = {"id": gid, "cand": cand, "pan": round(pan, 1), "tilt": round(tilt, 1), "t": {"req": now}}
        self.last_glance = now
        self.pub_req.publish(String(data="glance:" + json.dumps({"id": gid, "pan": round(pan, 1), "tilt": round(tilt, 1)})))
        self.get_logger().info(f"GLANCE #{gid} 요청: 후보 {cand['dist'] * 100:.0f} cm ({x:+.2f}, {y:+.2f}) "
                               f"{cand['hits']} 번 봄 → 고개 ({pan:+.0f}, {tilt:+.0f})")

    # ---------- 힐끗 진행 ----------
    def tick(self):
        j = self.job
        if not j:
            return
        now, t = time.time(), j["t"]
        if now - t["req"] > 4.0:                                        # 사령관이 안 받았거나 너무 오래 → 버림
            self.get_logger().warn(f"GLANCE #{j['id']} 포기 (모드 {self.mode}, {now - t['req']:.1f} s)")
            self.job = None
            return
        if "mode" not in t:
            if self.mode == "GLANCE" and self.prof_glance and self.prof_glance.get("id") == j["id"]:
                t["mode"] = now
            return
        if "head" not in t:
            if self.head_cmd == (j["pan"], j["tilt"]):
                t["head"] = self.head_t
            return
        if now < t["head"] + self.settle or self.frame_t < t["head"] + self.settle or self.frame is None:
            return
        t["frame"] = self.frame_t
        m = self.frame
        a = np.frombuffer(bytes(m.data), np.uint8).reshape(m.height, m.step)[:, : m.width * 3].reshape(m.height, m.width, 3)
        bgr = a if m.encoding == "bgr8" else np.ascontiguousarray(a[:, :, ::-1])
        dets = self.yolo.detect(bgr)
        t["yolo"] = time.time()
        cx, cy = gl.to_robot(self.pose, j["cand"]["wx"], j["cand"]["wy"])
        best = None
        for name, sc, bx, by, bw, bh in dets:
            fl = ground.pixels_to_floor([[bx + bw / 2, by + bh]], self.K, self.D, j["pan"], j["tilt"])[0]
            if np.isfinite(fl).all() and math.hypot(fl[0] - cx, fl[1] - cy) < self.match_r and (best is None or sc > best[1]):
                best = (name, sc, bx, by, bw, bh)
        name, conf = (best[0], round(float(best[1]), 3)) if best else ("?", 0.0)
        ms = {k: int((t[k] - t["req"]) * 1000) for k in ("mode", "head", "frame", "yolo")}
        self.cands.remember(j["cand"]["wx"], j["cand"]["wy"], now, name, conf)
        self.pub_seen.publish(String(data=json.dumps({"id": j["id"], "name": name, "conf": conf, "wx": round(j["cand"]["wx"], 3),
                                                      "wy": round(j["cand"]["wy"], 3), "dist": round(j["cand"]["dist"], 3), "ms": ms})))
        self.pub_ev.publish(String(data=json.dumps({"type": "glanced", "id": j["id"]})))
        others = ", ".join(f"{n} {s:.2f}" for n, s, *_ in sorted(dets, key=lambda d: -d[1])[:4]) or "없음"
        self.get_logger().info(f"GLANCE #{j['id']} → {name} {conf:.2f} | 시간 ms 사령관 {ms['mode']} 고개 {ms['head']} "
                               f"영상 {ms['frame']} YOLOE {ms['yolo']} (YOLOE만 {int((t['yolo'] - t['frame']) * 1000)}) | 화면 전체: {others}")
        img = bgr.copy()
        for n, s, bx, by, bw, bh in dets:
            cv2.rectangle(img, (int(bx), int(by)), (int(bx + bw), int(by + bh)), (0, 200, 0) if best and n == best[0] else (160, 160, 160), 2)
            cv2.putText(img, f"{n} {s:.2f}", (int(bx), max(12, int(by) - 4)), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 200, 0), 1)
        c = ground.floor_to_pixels([[cx, cy]], self.K, self.D, j["pan"], j["tilt"])[0]
        if np.isfinite(c).all():
            cv2.drawMarker(img, (int(c[0]), int(c[1])), (0, 0, 255), cv2.MARKER_CROSS, 18, 2)
        cv2.imwrite(os.path.join(OUT, time.strftime("%m%d_%H%M%S") + f"_{j['id']}_{name.replace(' ', '_')}.jpg"), img)
        self.job = None


def main():
    rclpy.init()
    rclpy.spin(Inspect())


if __name__ == "__main__":
    main()
