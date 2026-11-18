#!/usr/bin/env python3
"""glance_node — "안경 쓰기" 노드 (11-18). 바닥 후보가 새로우면 사령관에 힐끗 요청 → 고개가 가면 선명한 영상 2 장 → YOLO26 → (없으면) YOLOE.
  (처음 이름 inspect.py 는 파이썬 기본 모듈 inspect 를 가려서 같은 폴더 노드가 다 죽음 → 바꿈)

  판단은 glance.py (순수, tests/glance_sim.py). v2 (11-18 실물 16 장 보고 Jasper 와):
    - 쿨다운 없음 → **새로운 것만** (Candidates 기억 켜져 있는 동안, 거리 비례 반경). 같은 쓰레기통 5 번 보던 문제
    - 멈춤 대기 = settle_base + 각도 × settle_per_deg (90° 면 ~0.8 s). 큰 각도에서 흐린 사진 (1·2·9·13) 문제
    - **흐린 프레임 버림** (라플라시안 분산 < 평소의 sharp_ratio) + 멈춘 뒤 첫 장 버림 (자동 노출) → 선명한 frames (2) 장
    - **YOLO26 (COCO) 먼저** — chair·bed·cell phone·teddy bear 등은 COCO 가 더 잘 봄. 후보 자리에 coco_on 넘으면 끝.
      아니면 **YOLOE (COCO 에 없는 단어만)** — yoloe_on (0.25) 넘으면 이름, maybe (0.15) 넘으면 "이름?"
      (11-18 v4: 빈 바닥을 toy 0.18 로 = 오탐 → 0.15 → 0.25)
    - v4 마스크 (YOLOE 는 seg 모델): 이름은 **후보 십자가 마스크 위 (여유 mask_px)** 일 때. 마스크 아래 윤곽 → ground.py → **바닥 발자국**
      (위에서 본 모양) → 위치 = 발자국에서 로봇에 제일 가까운 점, semantic/seen 에 발자국 점들. YOLO26 (마스크 없음) 은 상자 아래 35%
    - 이름 맞추기: 후보 점 (화면) 이 상자 **아래쪽 35%** 안 (여유 margin px), 같은 이름이면 작은 상자 — 바닥 좌표 비교는 1 m 넘으면
      오차가 커서 (7 번 발닦개) 바꿈, 상자 전체로 하면 큰 침대가 앞 빈 바닥까지 가져감 (v2 9 번)
    - 본 곳은 켜져 있는 동안 계속 기억 (Jasper: 30 분일 이유 없음). 물건이 옮겨진 건 나중에 지도 이름표 층에서
    - 상자 투표·다듬기: 같은 이름이 2 장에서 → 평균 점수·평균 상자. 한 장에서만 튄 건 버림
    - 가운데에서 본 상자 아래 끝 → 위치를 다시 계산해 기억 (가장자리에 걸쳤던 후보도 여기서 정확해짐)
  입력  floor/blobs, neri/profile, pantilt/cmd, camera/image_raw, odom
  출력  neri/request "glance:{id,pan,tilt}", neri/event glanced, semantic/seen {"id","name","conf","model","wx","wy","ms"}
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
        self.n_frames, self.sharp_ratio = p("frames", 2).value, p("sharp_ratio", 0.5).value
        self.coco_on, self.yoloe_on, self.maybe = p("coco_on", 0.35).value, p("yoloe_on", 0.25).value, p("maybe", 0.15).value
        self.margin, self.gap = p("margin_px", 20).value, p("gap_s", 1.0).value
        self.bottom = p("bottom_frac", 0.35).value         # 후보 점이 상자 아래 이 비율 안이어야 그 물건 (마스크 없을 때)
        self.mask_px = p("mask_px", 15).value               # 후보 점이 마스크에서 이만큼 안이면 그 물건 (마스크 있을 때)
        self.yolo26 = Yolo(os.path.expanduser(p("coco_model", "~/models/yolo26n.onnx").value), size=320, conf=0.2)
        self.yoloe = Yolo(os.path.expanduser(p("yoloe_model", "~/models/yoloe-26s-seg.onnx").value), size=320, conf=self.maybe)
        self.yoloe_words = [n for n in self.yoloe.names if n not in COCO_SET]
        self.K, self.D, self.size = ground.load_calib()
        self.cands = gl.Candidates(max_dist=p("max_dist", 1.2).value, memory_s=float("inf"))   # 11-18 Jasper: 켜져 있는 동안 계속 기억
        self.mode, self.prof_glance = "IDLE", None
        self.pose = (0.0, 0.0, 0.0)
        self.head_cmd, self.head_t = (0.0, 10.0), 0.0
        self.sharp_hist = collections.deque(maxlen=40)       # 고개 멈춘 지 오래된 영상들의 선명도 (평소 값)
        self.job, self.next_id, self.end_t = None, 1, 0.0
        self.push = None
        port = p("stream_port", 0).value
        if port:
            sys.path.insert(0, os.path.join(HERE, "..", "..", "tools", "lab"))
            from face_lab import serve
            self.push = serve(port)
        os.makedirs(OUT, exist_ok=True)
        self.create_subscription(String, "floor/blobs", self.on_blobs, 10)
        self.create_subscription(String, "neri/profile", self.on_profile, 10)
        self.create_subscription(Float32MultiArray, "pantilt/cmd", self.on_head, 10)
        self.create_subscription(Image, "camera/image_raw", self.on_img, qos_profile_sensor_data)
        self.create_subscription(Odometry, "odom", self.on_odom, 20)
        self.pub_req = self.create_publisher(String, "neri/request", 10)
        self.pub_ev = self.create_publisher(String, "neri/event", 10)
        self.pub_seen = self.create_publisher(String, "semantic/seen", 10)
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
        cand = self.cands.step(now, tuple(d["pose"]), [(b["x"], b["y"], b["area"], b.get("edge", False)) for b in d["blobs"]],
                               visible=self.visible_fn(pan0, tilt0))
        if not cand:
            return
        x, y = gl.to_robot(self.pose, cand["wx"], cand["wy"])          # 지금 위치 기준 (걷는 중이면 조금 바뀜)
        pan, tilt = gl.head_for(x, y)
        gid, self.next_id = self.next_id, self.next_id + 1
        turn = max(abs(pan - self.head_cmd[0]), abs(tilt - self.head_cmd[1]))
        self.job = {"id": gid, "cand": cand, "pan": round(pan, 1), "tilt": round(tilt, 1), "turn": turn,
                    "settle": self.settle_base + self.settle_deg * turn, "t": {"req": now}, "frames": [], "skipped": 0}
        self.pub_req.publish(String(data="glance:" + json.dumps({"id": gid, "pan": round(pan, 1), "tilt": round(tilt, 1)})))
        self.get_logger().info(f"GLANCE #{gid} 요청: 후보 {cand['dist'] * 100:.0f} cm ({x:+.2f}, {y:+.2f}) {cand['hits']} 번 봄"
                               f"{' (가장자리)' if cand['edge'] else ''} → 고개 ({pan:+.0f}, {tilt:+.0f}), 돌 각도 {turn:.0f}°")

    def on_img(self, m):
        now = time.time()
        j = self.job
        if not j:
            if now - self.head_t > 1.5:                                 # 고개 오래 멈춤 → 평소 선명도 배우기
                self.sharp_hist.append(sharpness(to_bgr(m)))
            return
        t = j["t"]
        if "mode" not in t:
            if self.mode == "GLANCE" and self.prof_glance and self.prof_glance.get("id") == j["id"]:
                t["mode"] = now
            elif now - t["req"] > 2.0:
                self.give_up("사령관이 안 받음")
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
        bgr = to_bgr(m)
        sh = sharpness(bgr)
        base = float(np.median(self.sharp_hist)) if len(self.sharp_hist) >= 5 else 0.0
        if "first" not in t:                                            # 멈춘 뒤 첫 장 버림 (자동 노출 바뀌는 중)
            t["first"] = now
            return
        if base and sh < self.sharp_ratio * base:
            j["skipped"] += 1                                           # 흐림 → 버리고 다음 장
            return
        j["frames"].append((bgr, sh))
        if len(j["frames"]) >= self.n_frames:
            t["frames"] = now
            self.decide(j, base)

    @staticmethod
    def width(foot):
        """발자국 폭 = 가장 먼 두 점 거리 (m)"""
        f = np.asarray(foot)
        return float(max(np.hypot(*(a - b)) for a in f for b in f)) if len(f) > 1 else 0.0

    def give_up(self, why):
        self.get_logger().warn(f"GLANCE #{self.job['id']} 포기: {why}")
        self.job, self.end_t = None, time.time()

    # ---------- 이름 정하기 ----------
    def vote(self, model, words, frames, cpx, on):
        """프레임마다 후보 점 (cpx) 이 그 물건인 검출 → 이름별 [(점수, 상자, 마스크)]. 2 장 (또는 받은 장 수) 에서 나온 이름 중 평균 점수 최고.
           마스크가 있으면 (YOLOE seg): 후보 점에서 마스크까지 거리 ≤ mask_px. 없으면 (YOLO26): 상자 아래쪽 bottom_frac 안"""
        per = collections.defaultdict(list)
        alldet = []
        cx, cy = int(round(cpx[0])), int(round(cpx[1]))
        for bgr, _ in frames:
            dets = model.detect(bgr, masks=True)
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
        best, dets26 = self.vote(self.yolo26, None, j["frames"], cpx, self.coco_on)
        t["yolo26"] = time.time()
        model, detsE = "yolo26", []
        if not best or best[1] < self.coco_on:
            bestE, detsE = self.vote(self.yoloe, set(self.yoloe_words), j["frames"], cpx, self.yoloe_on)
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
        foot = []
        if best and best[3] is not None:                                # 마스크 → 바닥 발자국 → 위치 = 로봇에 제일 가까운 발자국 점
            fp = self.footprint(best[3], j["pan"], j["tilt"])
            if len(fp):
                i = int(np.argmin(np.hypot(fp[:, 0], fp[:, 1])))
                wx, wy = gl.to_world(self.pose, fp[i, 0], fp[i, 1])
                foot = [[round(v, 3) for v in gl.to_world(self.pose, x, y)] for x, y in fp]
        elif best:                                                      # 마스크 없음: 가운데에서 본 상자 아래 끝으로 위치 다시
            bx, by, bw, bh = best[2]
            fl = ground.pixels_to_floor([[bx + bw / 2, by + bh]], self.K, self.D, j["pan"], j["tilt"])[0]
            if np.isfinite(fl).all() and math.hypot(fl[0], fl[1]) < 2.0:
                wx, wy = gl.to_world(self.pose, fl[0], fl[1])
        self.cands.remember(j["cand"]["wx"], j["cand"]["wy"], now, name, conf, pose=self.pose)
        self.cands.remember(wx, wy, now, name, conf, pose=self.pose)
        r = t["req"]
        ms = {k: int((t[k] - r) * 1000) for k in ("mode", "head", "frames", "yolo26", "yoloe") if k in t}
        self.pub_seen.publish(String(data=json.dumps({"id": j["id"], "name": name, "conf": round(conf, 3), "model": model,
                                                      "wx": round(wx, 3), "wy": round(wy, 3), "footprint": foot, "ms": ms})))
        self.pub_ev.publish(String(data=json.dumps({"type": "glanced", "id": j["id"]})))
        top = lambda ds: ", ".join(f"{d[0]} {d[1]:.2f}" for d in sorted(ds, key=lambda d: -d[1])[:3]) or "-"
        self.get_logger().info(
            f"GLANCE #{j['id']} → {name} {conf:.2f} ({model}) | ms 사령관 {ms.get('mode')} 고개 {ms.get('head')} "
            f"영상 {ms.get('frames')} (대기 {j['settle']:.2f} s, 흐림 버림 {j['skipped']}) YOLO26 {ms.get('yolo26')}"
            + (f" YOLOE {ms['yoloe']}" if "yoloe" in ms else "")
            + (f" | 발자국 {len(foot)} 점, 폭 {self.width(foot) * 100:.0f} cm" if foot else "") + f" | 화면: {top(dets26[-1])}"
            + (f" / E: {top(detsE[-1])}" if detsE else ""))
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
        img = cv2.addWeighted(ov, 0.4, img, 0.6, 0)
        if best:
            bx, by, bw, bh = best[2]
            cv2.rectangle(img, (int(bx), int(by)), (int(bx + bw), int(by + bh)), (0, 220, 0), 2)
        if np.isfinite(cpx).all():
            cv2.drawMarker(img, (int(cpx[0]), int(cpx[1])), (0, 0, 255), cv2.MARKER_CROSS, 18, 2)
        cv2.putText(img, f"#{j['id']} {name} {conf:.2f} {model}  {ms.get('yoloe', ms.get('yolo26'))} ms  sharp {j['frames'][-1][1]:.0f}/{base:.0f}",
                    (6, img.shape[0] - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.imwrite(os.path.join(OUT, time.strftime("%m%d_%H%M%S") + f"_{j['id']}_{name.replace(' ', '_').replace('?', 'q')}.jpg"), img)
        if self.push:
            self.push(img)
        self.job, self.end_t = None, time.time()


def main():
    rclpy.init()
    rclpy.spin(GlanceNode())


if __name__ == "__main__":
    main()
