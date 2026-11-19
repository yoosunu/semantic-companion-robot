#!/usr/bin/env python3
"""glance_node — "안경 쓰기" 노드 (11-18). 바닥 후보가 새로우면 사령관에 힐끗 요청 → 고개가 가면 선명한 영상 2 장 → YOLO26 → (없으면) YOLOE.
  (처음 이름 inspect.py 는 파이썬 기본 모듈 inspect 를 가려서 같은 폴더 노드가 다 죽음 → 바꿈)

  판단은 glance.py (순수, tests/glance_sim.py). v2 (11-18 실물 16 장 보고 Jasper 와):
    - 쿨다운 없음 → **새로운 것만** (Candidates 기억 켜져 있는 동안, 거리 비례 반경). 같은 쓰레기통 5 번 보던 문제
    - 멈춤 대기 = settle_base + 각도 × settle_per_deg (90° 면 ~0.8 s). 큰 각도에서 흐린 사진 (1·2·9·13) 문제
    - **흐린 프레임 버림** + 멈춘 뒤 첫 장 버림 (자동 노출) → 선명한 frames (2) 장. 11-19: 기준을 '평소' → **같은 장면에서 가장 선명한 것의
      sharp_ratio (0.6)** 로 (glance.pick_sharp). 흰 벽 쪽은 원래 밋밋해서 전부 흐림 → 4 초 포기 7 번. 같은 자리 2 번 포기면 '?' 로 기억
    - **YOLO26 (COCO) 먼저** — chair·bed·cell phone·teddy bear 등은 COCO 가 더 잘 봄. 후보 자리에 coco_on 넘으면 끝.
      아니면 **YOLOE (COCO 에 없는 단어만)** — yoloe_on (0.25) 넘으면 이름, maybe (0.15) 넘으면 "이름?"
      (11-18 v4: 빈 바닥을 toy 0.18 로 = 오탐 → 0.15 → 0.25)
    - v4 마스크 (YOLOE 는 seg 모델): 이름은 **후보 십자가 마스크 위 (여유 mask_px)** 일 때. 마스크 아래 윤곽 → ground.py → **바닥 발자국**
      (위에서 본 모양) → 위치 = 발자국에서 로봇에 제일 가까운 점, semantic/seen 에 발자국 점들. YOLO26 (마스크 없음) 은 상자 아래 35%
    - v5 (11-19): YOLO26 도 seg (yolo26n-seg) → COCO 도 마스크로 이름 (#4 책상 밑 쓰레기통을 큰 chair 상자가 가져감).
      **자기 몸 거르기**: ground.body_mask (차체·LiDAR 상자를 고개 자세로 화면에 그림) 에 마스크가 body_frac (0.5) 이상 걸치면 버림
      (몸통 suitcase 0.33 → 96 %, LiDAR chair 0.26 → 100 %, 진짜 물건 0 %). **원본 프레임 저장** ~/glance_log/raw/ (+ json) → 오프라인 시뮬용
    - 11-19 의자 시험 (ground_snap chair_1m·60cm, 9 조합): n 은 의자가 다 보여도 0.15~0.66 들쭉날쭉, **s-seg 320 은 0.68~0.93**
      (480·640 은 오히려 나쁨) → 기본 **yolo26s-seg** (Pi: YOLO26 단계 2 장 ~0.6 s, n-seg ~0.25 s). Jasper: 핸드폰 아님, 눈이 잘 봐야
    - **덤 보기** (11-19): 십자 = '바닥 감지가 찍은 그 점의 정체' 장부 (그대로). 같은 화면의 다른 물건도 extra_on (0.5) 이상 + 두 장 모두면
      각자 마스크 발자국 위치로 semantic/seen 의 extras 에. 잘리지 않은 건 그 자리도 '봤음' 으로 기억 (바닥 후보로 또 안 봄)
    - 이름 맞추기: 후보 점 (화면) 이 상자 **아래쪽 35%** 안 (여유 margin px), 같은 이름이면 작은 상자 — 바닥 좌표 비교는 1 m 넘으면
      오차가 커서 (7 번 발닦개) 바꿈, 상자 전체로 하면 큰 침대가 앞 빈 바닥까지 가져감 (v2 9 번)
    - 본 곳은 켜져 있는 동안 계속 기억 (Jasper: 30 분일 이유 없음). 물건이 옮겨진 건 나중에 지도 이름표 층에서
    - 상자 투표·다듬기: 같은 이름이 2 장에서 → 평균 점수·평균 상자. 한 장에서만 튄 건 버림
    - 가운데에서 본 상자 아래 끝 → 위치를 다시 계산해 기억 (가장자리에 걸쳤던 후보도 여기서 정확해짐)
  입력  floor/blobs, neri/profile, pantilt/cmd, camera/image_raw, odom
  출력  neri/request "glance:{id,pan,tilt}", neri/event glanced, semantic/seen {"id","name","conf","model","wx","wy","footprint","outline","extras","ms"} (좌표 = odom)
  확인용: -p stream_port:=8086 → 마지막 힐끗 장면 (상자·후보 십자·투표·시간). 사진 ~/glance_log/ (집 안 → repo 에 안 올림)
  파이썬: onnxruntime → ~/yolo_venv/bin/python
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
from yolo_onnx import NAMES as COCO, Yolo  # noqa: E402

cv2.setNumThreads(1)
OUT = os.path.expanduser("~/glance_log")
COCO_SET = {n.replace("_", " ") for n in COCO}
MOVERS = {"person", "cat", "dog", "bird"}             # 덤 보기에서 자리 기억 안 함 (움직이는 것)


def to_bgr(m):
    a = np.frombuffer(bytes(m.data), np.uint8).reshape(m.height, m.step)[:, : m.width * 3].reshape(m.height, m.width, 3)
    return a if m.encoding == "bgr8" else np.ascontiguousarray(a[:, :, ::-1])


def sharpness(bgr):
    g = cv2.cvtColor(cv2.resize(bgr, (160, 120)), cv2.COLOR_BGR2GRAY)
    return float(cv2.Laplacian(g, cv2.CV_64F).var())


class GlanceNode(Node):
    def __init__(self):
        super().__init__("glance")
        p = self.declare_parameter
        self.settle_base, self.settle_deg = p("settle_base", 0.35).value, p("settle_per_deg", 0.005).value
        self.n_frames, self.sharp_ratio = p("frames", 2).value, p("sharp_ratio", 0.6).value
        self.extra, self.max_fail = p("frames_extra", 3).value, p("max_fail", 2).value
        self.focus_max, self.focus_min_d = p("focus_max", 15.0).value, p("focus_min_d", 0.45).value   # 따라보기 (11-20)
        self.track_deg = p("track_deg", 1.5).value          # 따라가기: 물건 각도가 이만큼 바뀌면 고개를 따라 돌림 (초당 5 번 계산)
        self.create_timer(0.2, self.track)
        self.hazards = []
        self.coco_on, self.yoloe_on, self.maybe = p("coco_on", 0.35).value, p("yoloe_on", 0.25).value, p("maybe", 0.15).value
        self.margin, self.gap = p("margin_px", 20).value, p("gap_s", 3.0).value
        self.bottom = p("bottom_frac", 0.35).value         # 후보 점이 상자 아래 이 비율 안이어야 그 물건 (마스크 없을 때)
        self.mask_px = p("mask_px", 15).value               # 후보 점이 마스크에서 이만큼 안이면 그 물건 (마스크 있을 때)
        self.body_frac = p("body_frac", 0.5).value          # 마스크 (없으면 상자) 가 로봇 몸 자리에 이 비율 이상이면 자기 몸 → 버림
        self.body_cache = {}
        self.extra_on, self.extra_dist = p("extra_on", 0.5).value, p("extra_dist", 2.0).value   # 덤 보기 (11-19)
        self.yolo26 = Yolo(os.path.expanduser(p("coco_model", "~/models/yolo26s-seg.onnx").value), size=320, conf=0.2)
        self.yoloe = Yolo(os.path.expanduser(p("yoloe_model", "~/models/yoloe-26s-seg.onnx").value), size=320, conf=self.maybe)
        self.yoloe_words = [n for n in self.yoloe.names if n not in COCO_SET]
        self.K, self.D, self.size = ground.load_calib()
        self.cands = gl.Candidates(max_dist=p("max_dist", 1.2).value, memory_s=float("inf"), need=p("cand_need", 3).value)
        #   11-18 Jasper: 켜져 있는 동안 계속 기억. 11-20: 2 → 3 번 봐야 후보, 힐끗 사이 1 → 3 s (8 번 중 빈 바닥 3·코앞 1 = 부산스러움)
        self.near_edge = p("near_edge_m", 0.45).value       # 11-20: 이 안의 화면 끝 후보는 버림 (켜자마자 (0.2, ±0.34) 매번 = 어안 구석)
        self.yoloe_on_focus = p("yoloe_focus_on", 0.4).value   # 11-20 Jasper: YOLOE 는 원래 낮게 나옴 → 따라보기 확신 문턱 0.4 (COCO 는 extra_on 0.5)
        self.mode, self.prof_glance = "IDLE", None
        self.pose = (0.0, 0.0, 0.0)
        self.head_cmd, self.head_t = (0.0, 10.0), 0.0
        self.job, self.next_id, self.end_t = None, 1, 0.0
        self.push = None
        port = p("stream_port", 0).value
        if port:
            sys.path.insert(0, os.path.join(HERE, "..", "..", "tools", "lab"))
            from face_lab import serve
            self.push = serve(port)
        os.makedirs(OUT, exist_ok=True)
        self.create_subscription(String, "floor/blobs", self.on_blobs, 10)
        self.create_subscription(String, "/semantic/hazards", lambda m: setattr(self, "hazards", json.loads(m.data)), 10)
        self.create_subscription(String, "neri/profile", self.on_profile, 10)
        self.create_subscription(Float32MultiArray, "pantilt/cmd", self.on_head, 10)
        self.create_subscription(Image, "camera/image_raw", self.on_img, qos_profile_sensor_data)
        self.create_subscription(Odometry, "odom", self.on_odom, 20)
        self.pub_req = self.create_publisher(String, "neri/request", 10)
        self.pub_ev = self.create_publisher(String, "neri/event", 10)
        self.pub_seen = self.create_publisher(String, "semantic/seen", 10)
        self.pub_label = self.create_publisher(String, "semantic/label", 10)    # 따라보기 결과 하나 (11-20, 이름표용)
        self.get_logger().info(f"GLANCE 준비: YOLO26 (COCO) 먼저 ≥{self.coco_on} → YOLOE 단어 {len(self.yoloe_words)}개 "
                               f"({', '.join(self.yoloe_words)}) ≥{self.yoloe_on}" + (f", 스트림 {port}" if port else ""))

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

    def visible_fn(self, pan, tilt):
        W, H = self.size
        def vis(x, y):
            if math.hypot(x, y) > 1.5:
                return False
            px = ground.floor_to_pixels([[x, y]], self.K, self.D, pan, tilt)[0]
            return bool(np.isfinite(px).all() and 10 < px[0] < W - 10 and 10 < px[1] < H - 10)
        return vis

    def on_blobs(self, m):
        if self.job or not (self.mode == "IDLE" or self.mode.startswith("TASK:")) or time.time() - self.end_t < self.gap:
            return
        try:
            d = json.loads(m.data)
        except ValueError:
            return
        now = time.time()
        pan0, tilt0 = d.get("head", [0.0, 10.0])
        blobs = [b for b in d["blobs"] if not (b.get("edge") and math.hypot(b["x"], b["y"]) < self.near_edge)]
        cand = self.cands.step(now, tuple(d["pose"]), [(b["x"], b["y"], b["area"], b.get("edge", False)) for b in blobs],
                               visible=self.visible_fn(pan0, tilt0))
        if not cand:
            return
        x, y = gl.to_robot(self.pose, cand["wx"], cand["wy"])          # 지금 위치 기준 (걷는 중이면 조금 바뀜)
        pan, tilt = gl.head_for(x, y)
        gid, self.next_id = self.next_id, self.next_id + 1
        turn = max(abs(pan - self.head_cmd[0]), abs(tilt - self.head_cmd[1]))
        self.job = {"id": gid, "cand": cand, "pan": round(pan, 1), "tilt": round(tilt, 1), "turn": turn,
                    "settle": self.settle_base + self.settle_deg * turn, "t": {"req": now}, "frames": [], "pool": [], "skipped": 0}
        self.pub_req.publish(String(data="glance:" + json.dumps({"id": gid, "pan": round(pan, 1), "tilt": round(tilt, 1),
                                                                  "hold": self.focus_max + 3.0})))
        self.get_logger().info(f"GLANCE #{gid} 요청: 후보 {cand['dist'] * 100:.0f} cm ({x:+.2f}, {y:+.2f}) {cand['hits']} 번 봄"
                               f"{' (가장자리)' if cand['edge'] else ''} → 고개 ({pan:+.0f}, {tilt:+.0f}), 돌 각도 {turn:.0f}°")

    def on_img(self, m):
        now = time.time()
        j = self.job
        if not j:
            return
        t = j["t"]
        if "mode" not in t:
            if self.mode == "GLANCE" and self.prof_glance and self.prof_glance.get("id") == j["id"]:
                t["mode"] = now
            elif now - t["req"] > 2.0:
                self.give_up("사령관이 안 받음")
            return
        if j.get("views"):                                              # 따라보는 중 (11-20): 고개가 계속 따라가니 "멈추고 기다림" 대신
            if self.mode != "GLANCE":                                   #   명령이 0.15 s 안 바뀌고 목표와 2° 안이면 찍음 (흔들린 장은 흐림이 거름)
                self.give_up("힐끗 끝남 (사람·시간)")
                return
            a = j.get("aim", (j["pan"], j["tilt"]))
            if now - self.head_t < 0.15 or max(abs(self.head_cmd[0] - a[0]), abs(self.head_cmd[1] - a[1])) > 2.0:
                return
            j["pan"], j["tilt"] = self.head_cmd                         # 이 장의 고개 자세 (바닥 좌표 계산용)
            bgr = to_bgr(m)
            j["pool"].append((bgr, sharpness(bgr)))
            idx = gl.pick_sharp([s for _, s in j["pool"]], self.n_frames, self.sharp_ratio, self.extra)
            if idx is None:
                return
            j["frames"] = [j["pool"][i] for i in idx]
            j["skipped"] = len(j["pool"]) - len(idx)
            t["head"] = t["frames"] = now
            j["settle"] = 0.0
            self.decide(j, max(s for _, s in j["pool"]))
            return
        if "head" not in t:
            if self.head_cmd == (j["pan"], j["tilt"]):
                t["head"] = self.head_t
            elif now - t["mode"] > 1.0:
                self.give_up("고개 명령 안 옴")
            return
        if now < t["head"] + j["settle"]:
            return
        if self.mode != "GLANCE":
            self.give_up("힐끗 끝남 (사람·시간)")
            return
        if "first" not in t:                                            # 멈춘 뒤 첫 장 버림 (자동 노출 바뀌는 중)
            t["first"] = now
            return
        bgr = to_bgr(m)
        j["pool"].append((bgr, sharpness(bgr)))
        idx = gl.pick_sharp([s for _, s in j["pool"]], self.n_frames, self.sharp_ratio, self.extra)
        if idx is None:
            return                                                      # 이 장면에서 선명한 장이 모자람 → 다음 장
        j["frames"] = [j["pool"][i] for i in idx]
        j["skipped"] = len(j["pool"]) - len(idx)
        t["frames"] = now
        self.decide(j, max(s for _, s in j["pool"]))

    @staticmethod
    def width(foot):
        """발자국 폭 = 가장 먼 두 점 거리 (m)"""
        f = np.asarray(foot)
        return float(max(np.hypot(*(a - b)) for a in f for b in f)) if len(f) > 1 else 0.0

    def give_up(self, why):
        j = self.job
        gone = self.cands.fail(j["cand"]["wx"], j["cand"]["wy"], time.time(), pose=self.pose, limit=self.max_fail)
        self.get_logger().warn(f"GLANCE #{j['id']} 포기: {why} (받은 장 {len(j['pool'])}, 본 {j.get('views', 0)} 번)"
                               + (f" → 같은 자리 {self.max_fail} 번째: '?' 로 기억, 다시 안 봄" if gone else ""))
        self.pub_ev.publish(String(data=json.dumps({"type": "glanced", "id": j["id"]})))   # 사령관이 hold 동안 붙잡지 않게
        self.job, self.end_t = None, time.time()

    # ---------- 이름 정하기 ----------
    def body(self, pan, tilt, h, w):
        """고개 자세별 로봇 몸 자리 (한 번 계산 ~5 ms 후 기억)"""
        k = (round(pan, 1), round(tilt, 1), h, w)
        if k not in self.body_cache:
            self.body_cache[k] = ground.body_mask(self.K, self.D, pan, tilt, w, h)
        return self.body_cache[k]

    def is_body(self, d, body):
        if len(d) > 6:
            m = d[6]
        else:
            m = np.zeros_like(body)
            x, y, w, h = (int(round(v)) for v in d[2:6])
            m[max(0, y):y + h, max(0, x):x + w] = True
        n = m.sum()
        return n > 0 and (m & body).sum() >= self.body_frac * n

    def vote(self, model, words, frames, cpx, on, body=None):
        """프레임마다 후보 점 (cpx) 이 그 물건인 검출 → 이름별 [(점수, 상자, 마스크)]. 2 장 (또는 받은 장 수) 에서 나온 이름 중 평균 점수 최고.
           마스크가 있으면 (YOLOE seg): 후보 점에서 마스크까지 거리 ≤ mask_px. 없으면 (YOLO26): 상자 아래쪽 bottom_frac 안"""
        per = collections.defaultdict(list)
        alldet = []
        cx, cy = int(round(cpx[0])), int(round(cpx[1]))
        for bgr, _ in frames:
            dets = model.detect(bgr, masks=True)
            if body is not None:
                dets = [d for d in dets if not self.is_body(d, body)]        # 자기 몸 (몸통·LiDAR) 은 처음부터 뺌
            alldet.append(dets)
            got = {}
            for d in dets:
                name, sc, bx, by, bw, bh = d[:6]
                mask = d[6] if len(d) > 6 else None
                if words is not None and name not in words:
                    continue
                if mask is not None:
                    ys, xs = np.nonzero(mask[max(0, cy - self.mask_px):cy + self.mask_px + 1, max(0, cx - self.mask_px):cx + self.mask_px + 1])
                    hit = len(ys) > 0
                else:
                    # 후보 점 = 물건이 바닥에 닿은 곳 → 상자 **아래쪽 끝 근처** 여야 그 물건 (11-18 9번: 큰 침대 상자가 앞 빈 바닥까지 가져감)
                    low = by + bh * (1 - self.bottom)
                    hit = bx - self.margin <= cpx[0] <= bx + bw + self.margin and low - self.margin <= cpx[1] <= by + bh + self.margin
                if hit:
                    area = bw * bh
                    if name not in got or area < got[name][3]:       # 같은 이름이면 작은 것 (더 정확히 감싼 것)
                        got[name] = (sc, (bx, by, bw, bh), mask, area)
            for name, v in got.items():
                per[name].append(v[:3])
        need = min(2, len(frames))
        best = None
        for name, v in per.items():
            if len(v) < need:
                continue                                                # 한 장에서만 튐 → 버림
            sc = sum(x[0] for x in v) / len(frames)
            box = tuple(np.mean([x[1] for x in v], axis=0))
            if best is None or sc > best[1]:
                best = (name, sc, box, v[-1][2])                        # 마스크는 마지막 장 것
        return best, alldet

    def extras(self, j, dets_all, chosen):
        """덤 보기 (11-19 Jasper: 십자 물건이 아니어도 확실히 보인 건 기록). 십자 규칙은 '그 바닥 점의 정체' 장부로 그대로 두고,
           같은 화면의 다른 물건 중 extra_on (0.5, Jasper: 0.5 못 넘으면 못 본 것) 이상 + 모든 장에서 같은 이름 → 각자 마스크 발자국의
           가장 가까운 점 (extra_dist 2 m 안). 화면 아래 끝에 잘린 마스크는 cut (바닥 닿은 점이 안 보여 위치를 덜 믿음 → 기억은 안 함)"""
        out = []
        prev = [{d[0] for d in ds if d[1] >= self.extra_on} for ds in dets_all[:-1]]
        for d in dets_all[-1]:
            if d[1] < self.extra_on or len(d) <= 6 or (chosen is not None and d[6] is chosen):
                continue
            if any(d[0] not in s for s in prev):
                continue                                                # 한 장에서만 → 버림
            fp = self.footprint(d[6], j["pan"], j["tilt"])
            if not len(fp):
                continue
            k = int(np.argmin(np.hypot(fp[:, 0], fp[:, 1])))
            x, y = float(fp[k, 0]), float(fp[k, 1])
            if math.hypot(x, y) > self.extra_dist:
                continue
            wx, wy = gl.to_world(self.pose, x, y)
            out.append({"name": d[0], "conf": round(float(d[1]), 3), "x": round(x, 3), "y": round(y, 3),
                        "wx": round(wx, 3), "wy": round(wy, 3), "cut": self.cut(d[6]), "cut_bottom": bool(d[6][-3:].any()), "box": [int(v) for v in d[2:6]],
                        "footprint": self.world(fp), "outline": self.world(self.outline(d[6], j["pan"], j["tilt"]))})
        return out

    @staticmethod
    def cut(mask, edge=3):
        """마스크가 화면 끝 (네 변 어디든) 에 닿음 = 일부만 보임. 11-20: 아래만 봤더니 좌우 끝에 잘린 발닦개가 삼각형·조각으로 찍힘"""
        return bool(mask[:edge].any() or mask[-edge:].any() or mask[:, :edge].any() or mask[:, -edge:].any())

    def outline(self, mask, pan, tilt):
        """마스크 바깥 윤곽 전체 → 바닥 (x, y) 로봇 좌표 m (최대 40 점). 납작한 물건 (발닦개) 은 윤곽 전체가 바닥 위라 이게 모양.
           떠 있는 물건엔 틀린 값 → 쓸지는 받는 쪽 (floor_hazard: 바닥형 + LiDAR 못 봄) 이 정함 (11-19)"""
        cs, _ = cv2.findContours(mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not cs:
            return np.zeros((0, 2))
        c = max(cs, key=cv2.contourArea).reshape(-1, 2).astype(float) + 0.5
        c = c[:: max(1, len(c) // 40)]
        fl = ground.pixels_to_floor(c, self.K, self.D, pan, tilt)
        fl = fl[np.isfinite(fl).all(1)]
        return fl[np.hypot(fl[:, 0], fl[:, 1]) < 2.5]

    def world(self, pts):
        return [[round(v, 3) for v in gl.to_world(self.pose, x, y)] for x, y in pts]

    def footprint(self, mask, pan, tilt):
        """마스크 아래 윤곽 (열마다 가장 아래 픽셀) → 바닥 (x, y) 로봇 좌표 m. 위에서 본 물건이 바닥에 닿은 모양"""
        cols = np.nonzero(mask.any(0))[0]
        if not len(cols):
            return np.zeros((0, 2))
        cols = cols[:: max(1, len(cols) // 24)]
        bottom = [(c + 0.5, np.nonzero(mask[:, c])[0].max() + 0.5) for c in cols]
        fl = ground.pixels_to_floor(bottom, self.K, self.D, pan, tilt)
        fl = fl[np.isfinite(fl).all(1)]
        return fl[np.hypot(fl[:, 0], fl[:, 1]) < 2.5]

    def decide(self, j, base):
        t, now = j["t"], time.time()
        cx, cy = gl.to_robot(self.pose, j["cand"]["wx"], j["cand"]["wy"])
        cpx = ground.floor_to_pixels([[cx, cy]], self.K, self.D, j["pan"], j["tilt"])[0]
        h, w = j["frames"][0][0].shape[:2]
        body = self.body(j["pan"], j["tilt"], h, w)
        stem = time.strftime("%m%d_%H%M%S") + f"_{j['id']}"
        os.makedirs(os.path.join(OUT, "raw"), exist_ok=True)              # 원본 (그림 없음) → 오프라인 시뮬
        for k, (bgr, _) in enumerate(j["frames"]):
            cv2.imwrite(os.path.join(OUT, "raw", f"{stem}_{k}.jpg"), bgr)
        json.dump({"id": j["id"], "pan": j["pan"], "tilt": j["tilt"], "cpx": [float(v) for v in cpx], "cand": j["cand"],
                   "pose": list(self.pose)}, open(os.path.join(OUT, "raw", stem + ".json"), "w"), default=float)
        best, dets26 = self.vote(self.yolo26, None, j["frames"], cpx, self.coco_on, body)
        t["yolo26"] = time.time()
        model, detsE = "yolo26", []
        if not best or best[1] < self.coco_on:
            bestE, detsE = self.vote(self.yoloe, set(self.yoloe_words), j["frames"], cpx, self.yoloe_on, body)
            t["yoloe"] = time.time()
            if bestE and bestE[1] >= self.maybe and (not best or bestE[1] >= best[1] * 0.5):
                best, model = bestE, "yoloe"
        on = self.coco_on if model == "yolo26" else self.yoloe_on
        if best and best[1] >= on:
            name, conf = best[0], best[1]
        elif best and best[1] >= self.maybe:
            name, conf = best[0] + "?", best[1]
        else:
            name, conf, model = "?", 0.0, "-"
        wx, wy = j["cand"]["wx"], j["cand"]["wy"]
        foot, outl = [], []
        if best and best[3] is not None:                                # 마스크 → 바닥 발자국 → 위치 = 로봇에 제일 가까운 발자국 점
            fp = self.footprint(best[3], j["pan"], j["tilt"])
            if len(fp):
                i = int(np.argmin(np.hypot(fp[:, 0], fp[:, 1])))
                wx, wy = gl.to_world(self.pose, fp[i, 0], fp[i, 1])
                foot = self.world(fp)
            outl = self.world(self.outline(best[3], j["pan"], j["tilt"]))
        elif best:                                                      # 마스크 없음: 가운데에서 본 상자 아래 끝으로 위치 다시
            bx, by, bw, bh = best[2]
            fl = ground.pixels_to_floor([[bx + bw / 2, by + bh]], self.K, self.D, j["pan"], j["tilt"])[0]
            if np.isfinite(fl).all() and math.hypot(fl[0], fl[1]) < 2.0:
                wx, wy = gl.to_world(self.pose, fl[0], fl[1])
        self.cands.remember(j["cand"]["wx"], j["cand"]["wy"], now, name, conf, pose=self.pose)
        self.cands.remember(wx, wy, now, name, conf, pose=self.pose)
        ext = self.extras(j, dets26, best[3] if best and model == "yolo26" else None)
        if detsE:
            ext += self.extras(j, detsE, best[3] if best and model == "yoloe" else None)
        for e in ext:
            if not e["cut"] and e["name"] not in MOVERS:               # 사람·동물은 움직임 → 자리를 '봤음' 으로 묶지 않음
                self.cands.remember(e["wx"], e["wy"], now, e["name"], e["conf"], pose=self.pose)   # 이름 아는 자리 → 바닥 후보로 또 안 봄
        r = t["req"]
        ms = {k: int((t[k] - r) * 1000) for k in ("mode", "head", "frames", "yolo26", "yoloe") if k in t}
        self.pub_seen.publish(String(data=json.dumps({"id": j["id"], "name": name, "conf": round(conf, 3), "model": model,
                                                      "wx": round(wx, 3), "wy": round(wy, 3), "footprint": foot, "outline": outl,
                                                      "cut": bool(best and best[3] is not None and self.cut(best[3])), "extras": ext, "ms": ms})))
        top = lambda ds: ", ".join(f"{d[0]} {d[1]:.2f}" for d in sorted(ds, key=lambda d: -d[1])[:3]) or "-"
        self.get_logger().info(
            f"GLANCE #{j['id']} → {name} {conf:.2f} ({model}) | ms 사령관 {ms.get('mode')} 고개 {ms.get('head')} "
            f"영상 {ms.get('frames')} (대기 {j['settle']:.2f} s, 받은 {len(j['pool'])} 장 중 {len(j['frames'])}) YOLO26 {ms.get('yolo26')}"
            + (f" YOLOE {ms['yoloe']}" if "yoloe" in ms else "")
            + (f" | 발자국 {len(foot)} 점, 폭 {self.width(foot) * 100:.0f} cm" if foot else "") + f" | 화면: {top(dets26[-1])}"
            + (f" / E: {top(detsE[-1])}" if detsE else "")
            + (" | 덤: " + ", ".join(f"{e['name']} {e['conf']:.2f} ({e['x']:+.2f}, {e['y']:+.2f}){' 잘림' if e['cut'] else ''}" for e in ext) if ext else ""))
        img = j["frames"][-1][0].copy()
        ov = img.copy()
        for ds, col in ((dets26[-1], (200, 200, 0)), (detsE[-1] if detsE else [], (200, 0, 200))):
            for d in ds:
                n, sc, bx, by, bw, bh = d[:6]
                if len(d) > 6 and sc >= self.maybe:
                    ov[d[6]] = col                                      # 마스크 칠하기
                cv2.rectangle(img, (int(bx), int(by)), (int(bx + bw), int(by + bh)), col, 1)
                cv2.putText(img, f"{n} {sc:.2f}", (int(bx), max(12, int(by) - 3)), cv2.FONT_HERSHEY_SIMPLEX, 0.4, col, 1)
        if best and best[3] is not None:
            ov[best[3]] = (0, 220, 0)
        for e in ext:
            bx, by, bw, bh = e["box"]
            cv2.putText(img, "+" + e["name"], (bx, min(img.shape[0] - 24, by + bh + 14)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 220, 0), 1)
        cs, _ = cv2.findContours(body.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(img, cs, -1, (160, 160, 160), 1)                # 로봇 몸 자리 (회색 선)
        img = cv2.addWeighted(ov, 0.4, img, 0.6, 0)
        if best:
            bx, by, bw, bh = best[2]
            cv2.rectangle(img, (int(bx), int(by)), (int(bx + bw), int(by + bh)), (0, 220, 0), 2)
        if np.isfinite(cpx).all():
            cv2.drawMarker(img, (int(cpx[0]), int(cpx[1])), (0, 0, 255), cv2.MARKER_CROSS, 18, 2)
        cv2.putText(img, f"#{j['id']} {name} {conf:.2f} {model}  {ms.get('yoloe', ms.get('yolo26'))} ms  sharp {j['frames'][-1][1]:.0f}/{base:.0f}",
                    (6, img.shape[0] - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.imwrite(os.path.join(OUT, stem + f"_{name.replace(' ', '_').replace('?', 'q')}.jpg"), img)
        if self.push:
            self.push(img)
        self.focus_next(j, best, wx, wy, ext)

    def track(self):
        """따라가기 (11-20 Jasper): 따라보는 동안 초당 5 번, 지금 로봇 위치로 물건 각도를 다시 계산해 track_deg 넘게 바뀌면 고개를
           따라 돌림 → 주행 중에도 물건이 화면 안 (가운데 근처) 에 붙어 있음, 보는 사람도 "저걸 보는구나". YOLO 주기 (2.5 s) 와 따로"""
        j = self.job
        if not j or not j.get("views") or self.mode != "GLANCE":
            return
        x, y = gl.to_robot(self.pose, j["cand"]["wx"], j["cand"]["wy"])
        pan, tilt = gl.head_for(x, y)
        a = j.get("aim", (j["pan"], j["tilt"]))
        if max(abs(pan - a[0]), abs(tilt - a[1])) >= self.track_deg:
            self.aim(j, pan, tilt)

    def aim(self, j, pan, tilt):
        j["aim"] = (round(pan, 1), round(tilt, 1))
        self.pub_req.publish(String(data="glance:" + json.dumps({"id": j["id"], "pan": j["aim"][0], "tilt": j["aim"][1],
                                                                  "hold": self.focus_max + 3.0})))

    def hazard_near(self, wx, wy, r=0.4):
        """floor_hazard 표시 중 (odom 좌표) 가장 가까운 것 (r 안) 또는 None"""
        near = [h for h in self.hazards if "ox" in h and math.hypot(h["ox"] - wx, h["oy"] - wy) < r]
        return min(near, key=lambda h: math.hypot(h["ox"] - wx, h["oy"] - wy)) if near else None

    def focus_next(self, j, best, wx, wy, ext=()):
        """따라보기 (11-20 Jasper): 확신 못 한 건 고개로 그 물건만 따라가며 다시 봄 → 확신되면 끝 (glanced) → 다음 후보.
           확신 = glance.focus_verdict (이름 0.5 이상 두 번, 바닥형이면 통째 + 바닥 확인). 끝: 확신 / 빈 바닥 / focus_max 초과 /
           너무 가깝거나 멀어 못 봄 (고개 −30 이 바닥 ~0.45 m 까지)"""
        now = time.time()
        j["views"] = j.get("views", 0) + 1
        votes = j.setdefault("votes", {})
        if best and best[1] >= self.maybe:
            votes.setdefault(best[0], []).append(round(float(best[1]), 3))
        cw, ch = j["cand"]["wx"], j["cand"]["wy"]
        if best and best[3] is not None:
            j["full"] = j.get("full", False) or not self.cut(best[3])
            # 겨냥 옮기기: 아래 끝에 잘린 마스크는 안 됨 ("가장 가까운 점" = 끝선, 침대 → 0.15 m), 지금 목표 30 cm 안만 (11-20 #8: 문으로
            #   넘어감), 반만 옮김 (같은 물건인데 매번 몇 cm 씩 달라 고개가 꼼지락)
            if not best[3][-3:].any() and math.hypot(wx - cw, wy - ch) < 0.30:
                j["cand"] = dict(j["cand"], wx=(cw + wx) / 2, wy=(ch + wy) / 2)
        elif ext:
            # 11-20 실물: 십자가 발닦개 마스크를 살짝 빗나감 (덤으로는 rug 0.73 통째) → 표가 비어 빈 점만 따라봄.
            #   십자 30 cm 안에 덤 물건이 있으면 그걸로 겨냥을 옮김
            near = [e for e in ext if math.hypot(e["wx"] - cw, e["wy"] - ch) < 0.30 and not e.get("cut_bottom")]
            if near:
                e = min(near, key=lambda e: math.hypot(e["wx"] - cw, e["wy"] - ch))
                votes.setdefault(e["name"], []).append(round(float(e["conf"]), 3))
                j["full"] = j.get("full", False) or not e["cut"]
                j["cand"] = dict(j["cand"], wx=e["wx"], wy=e["wy"])
                wx, wy = e["wx"], e["wy"]
                self.get_logger().info(f"FOCUS #{j['id']} 십자 옆 {e['name']} {e['conf']:.2f} 로 겨냥 옮김 "
                                       f"({math.hypot(e['wx'] - cw, e['wy'] - ch) * 100:.0f} cm)")
        on_for = lambda n: self.yoloe_on_focus if n in self.yoloe_words else self.extra_on
        done, res, why = gl.focus_verdict(votes, j["views"], j.get("full", False), self.hazard_near(wx, wy), on=self.extra_on, on_for=on_for)
        took = now - j["t"]["req"]
        x, y = gl.to_robot(self.pose, j["cand"]["wx"], j["cand"]["wy"])
        d = math.hypot(x, y)
        if not done and (took > self.focus_max or not self.focus_min_d <= d <= 1.5):
            why = (f"시간 초과 {took:.0f} s" if took > self.focus_max else f"못 봄 (거리 {d:.2f} m)") + f" — {why}"
            done, res = True, res if res == "?" else res + "?"
        if done:
            self.pub_ev.publish(String(data=json.dumps({"type": "glanced", "id": j["id"]})))
            if res != "-":                                              # 이름표 (RViz, semantic_markers) — 빈 바닥은 안 띄움
                base = res.rstrip("?")
                v = votes.get(base, [])
                self.pub_label.publish(String(data=json.dumps({
                    "id": j["id"], "name": res, "conf": round(sum(v) / len(v), 3) if v else 0.0, "why": why, "views": j["views"],
                    "wx": round(j["cand"]["wx"], 3), "wy": round(j["cand"]["wy"], 3), "floor": base in gl.FLOOR_TYPES})))
            self.get_logger().info(f"FOCUS #{j['id']} 끝 → {res} | {why} | {j['views']} 번 봄, {took:.1f} s | 표 {votes}")
            self.job, self.end_t = None, now
            return
        pan, tilt = gl.head_for(x, y)
        j.update(frames=[], pool=[], skipped=0)
        for k in ("head", "first", "frames", "yolo26", "yoloe"):
            j["t"].pop(k, None)
        j["t"]["mode"] = now          # 11-20 버그: 안 바꾸면 "고개 명령 1 s 안에" 를 첫 보기 시각부터 재서 다시 겨누자마자 포기
        a = j.get("aim", (j["pan"], j["tilt"]))
        if max(abs(pan - a[0]), abs(tilt - a[1])) >= self.track_deg:   # 이후로는 track() 이 초당 5 번 따라감
            self.aim(j, pan, tilt)
        else:
            j.setdefault("aim", (j["pan"], j["tilt"]))
        self.get_logger().info(f"FOCUS #{j['id']} 다시 봄 ({why}) → 고개 ({pan:+.0f}, {tilt:+.0f}), 거리 {d:.2f} m, {took:.1f} s")


def main():
    rclpy.init()
    rclpy.spin(GlanceNode())


if __name__ == "__main__":
    main()
