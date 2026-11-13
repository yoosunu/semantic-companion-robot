#!/usr/bin/env python3
"""yolo_lab — YOLO 사물 실험실 (11-13). "바닥에서 본 내 침대를 YOLO 가 알아보나?" 확인용.

  YOLO (COCO 80 종류) 를 카메라 영상에 돌려서, 상자·이름·점수를 그린 실시간 영상을 브라우저로.
  COCO 에 있는 가구: bed(침대), chair(의자), couch(소파), dining table(식탁·책상), toilet, tv …
  가구는 굵은 주황 상자, 나머지는 얇은 회색. 1 s 마다 로그: 무엇이 몇 점으로 보였나.

  쓰기 (Pi, 카메라 노드가 켜져 있어야 함 — yolo_lab.sh 가 켜 줌)
    yolo_lab.py --stream              Mac 브라우저 http://scr-pi.local:8081 (집 Wi-Fi 안에서만, 시험할 때만)
    yolo_lab.py --stream --conf 0.15  약한 것도 보기 (기본 0.25)
    yolo_lab.py --model ~/models/yolo26n.onnx   YOLO26 (11-13: 11 보다 CPU 31% 빠름, NMS 없음) — 출력 모양 보고 알아서 처리
    yolo_lab.py --model ~/models/yoloe-26s-seg.onnx --conf 0.05   YOLOE: 단어로 찾기 (doormat rug cable sock power strip bed chair shoe,
                                  단어는 내보낼 때 정함. 점수가 원래 낮게 나옴 → 낮은 기준으로)
    yolo_lab.py --image 사진.png      카메라 없이 사진 한 장 (Mac/VM 시험용) → ~/yolo_lab/latest.jpg
  기록: ~/yolo_lab/log.csv (시각, 이름, 점수, 상자 크기 비율), 가구가 보이면 3 s 마다 그림 ~/yolo_lab/seen_XX.jpg
"""
import ast
import os
import sys
import time

import cv2
import numpy as np
import onnxruntime as ort

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from face_lab import frames, serve  # noqa: E402  (카메라 받기·MJPEG 스트림은 face_lab 것 그대로)

NAMES = ("person bicycle car motorcycle airplane bus train truck boat traffic_light fire_hydrant stop_sign "
         "parking_meter bench bird cat dog horse sheep cow elephant bear zebra giraffe backpack umbrella handbag "
         "tie suitcase frisbee skis snowboard sports_ball kite baseball_bat baseball_glove skateboard surfboard "
         "tennis_racket bottle wine_glass cup fork knife spoon bowl banana apple sandwich orange broccoli carrot "
         "hot_dog pizza donut cake chair couch potted_plant bed dining_table toilet tv laptop mouse remote keyboard "
         "cell_phone microwave oven toaster sink refrigerator book clock vase scissors teddy_bear hair_drier "
         "toothbrush").split()
FURNITURE = {"bed", "chair", "couch", "dining_table", "toilet", "tv", "refrigerator", "bench", "sink", "oven",
             "doormat", "rug", "cable", "sock", "power strip", "shoe"}      # 굵게 표시 (YOLOE 단어 포함)
OUT = os.path.expanduser("~/yolo_lab")
NAME = "YOLO"


class Yolo:
    def __init__(self, model, size=320, conf=0.25):
        self.sess = ort.InferenceSession(model, providers=["CPUExecutionProvider"])
        self.inp, self.size, self.conf = self.sess.get_inputs()[0].name, size, conf
        meta = self.sess.get_modelmeta().custom_metadata_map.get("names")      # 모델에 적힌 종류 이름 (YOLOE 는 우리가 준 단어들)
        self.names = [v for k, v in sorted(ast.literal_eval(meta).items())] if meta else list(NAMES)

    def detect(self, bgr):
        """→ [(이름, 점수, x, y, w, h)] 원본 좌표 (x,y = 왼쪽 위)"""
        h, w = bgr.shape[:2]
        s = self.size / max(h, w)
        nw, nh = int(round(w * s)), int(round(h * s))
        pad = np.full((self.size, self.size, 3), 114, np.uint8)
        px, py = (self.size - nw) // 2, (self.size - nh) // 2
        pad[py:py + nh, px:px + nw] = cv2.resize(bgr, (nw, nh))
        x = pad[:, :, ::-1].transpose(2, 0, 1)[None].astype(np.float32) / 255.0      # BGR → RGB, HWC → NCHW
        out = self.sess.run(None, {self.inp: x})[0][0]
        if out.shape[-1] == 6:                                                       # YOLO26 (nms=False): (300, 6) x1 y1 x2 y2 점수 종류, NMS 필요 없음
            return [(self.names[int(c)], float(sc), (x1 - px) / s, (y1 - py) / s, (x2 - x1) / s, (y2 - y1) / s)
                    for x1, y1, x2, y2, sc, c in out if sc >= self.conf]
        out = out.T                                                    # YOLO11 (2100, 84) / YOLOE (2100, 4+종류+32 마스크): cx cy w h + 점수 → NMS
        nc = len(self.names)
        cls = out[:, 4:4 + nc].argmax(1)
        sc = out[np.arange(len(out)), 4 + cls]
        keep = sc >= self.conf
        out, cls, sc = out[keep], cls[keep], sc[keep]
        boxes = [[(cx - bw / 2 - px) / s, (cy - bh / 2 - py) / s, bw / s, bh / s] for cx, cy, bw, bh in out[:, :4]]
        idx = cv2.dnn.NMSBoxes(boxes, sc.tolist(), self.conf, 0.45)
        return [(self.names[cls[i]], float(sc[i]), *boxes[i]) for i in np.array(idx).flatten()]


def draw(bgr, dets, ms):
    img = bgr.copy()
    for name, c, x, y, bw, bh in dets:
        f = name in FURNITURE
        col, th = ((0, 140, 255), 3) if f else ((170, 170, 170), 1)
        cv2.rectangle(img, (int(x), int(y)), (int(x + bw), int(y + bh)), col, th)
        cv2.putText(img, f"{name} {c:.2f}", (int(x) + 3, max(14, int(y) + 16)), cv2.FONT_HERSHEY_SIMPLEX, 0.55, col, 2 if f else 1)
    cv2.putText(img, f"{NAME} {ms:.0f} ms", (6, img.shape[0] - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    return img


def main():
    a = sys.argv[1:]
    arg = lambda k, d: type(d)(a[a.index(k) + 1]) if k in a else d
    global NAME
    model = os.path.expanduser(arg("--model", "~/models/yolo11n.onnx"))
    NAME = os.path.basename(model).replace(".onnx", "")
    yolo = Yolo(model, conf=arg("--conf", 0.25))
    os.makedirs(OUT, exist_ok=True)
    image = arg("--image", "")
    push = serve(8081) if "--stream" in a else None
    csv = open(os.path.join(OUT, "log.csv"), "a")
    if csv.tell() == 0:
        csv.write("t,name,conf,area\n")
    src = [cv2.imread(image)] if image else frames()
    t_log = t_save = 0.0
    k = 0
    for bgr in src:
        t0 = time.time()
        dets = yolo.detect(bgr)
        ms = (time.time() - t0) * 1000
        img = draw(bgr, dets, ms)
        now = time.time()
        area = bgr.shape[0] * bgr.shape[1]
        for d in dets:
            csv.write(f"{now:.2f},{d[0]},{d[1]:.3f},{d[4] * d[5] / area:.3f}\n")
        csv.flush()
        if push:
            push(img)
        if image or now - t_log >= 1.0:
            t_log = now
            seen = ", ".join(f"{d[0]} {d[1]:.2f}" for d in sorted(dets, key=lambda d: -d[1])) or "없음"
            print(f"YOLO {ms:.0f} ms | {seen}", flush=True)
            cv2.imwrite(os.path.join(OUT, "latest.jpg"), img)
        if not image and now - t_save >= 3.0 and any(d[0] in FURNITURE for d in dets):
            t_save = now
            cv2.imwrite(os.path.join(OUT, f"seen_{k % 30:02d}.jpg"), img)
            k += 1


if __name__ == "__main__":
    main()
