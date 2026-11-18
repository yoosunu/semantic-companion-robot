"""yolo_onnx — YOLO / YOLOE ONNX 추론 (노드용, 11-18). tools/lab/yolo_lab.py 의 Yolo 를 옮김 (lab 은 그대로).
  YOLO26 (nms=False, (300,6)) · YOLO11 (2100,84) · YOLOE (2100, 4+단어수+32 마스크) 를 출력 모양 보고 알아서 처리.
  YOLOE 단어는 ONNX 를 내보낼 때 정해짐 (메타데이터 names). 점수가 원래 낮게 나옴 → 낮은 기준 (0.05~0.1)
"""
import ast

import cv2
import numpy as np
import onnxruntime as ort

NAMES = ("person bicycle car motorcycle airplane bus train truck boat traffic_light fire_hydrant stop_sign "
         "parking_meter bench bird cat dog horse sheep cow elephant bear zebra giraffe backpack umbrella handbag "
         "tie suitcase frisbee skis snowboard sports_ball kite baseball_bat baseball_glove skateboard surfboard "
         "tennis_racket bottle wine_glass cup fork knife spoon bowl banana apple sandwich orange broccoli carrot "
         "hot_dog pizza donut cake chair couch potted_plant bed dining_table toilet tv laptop mouse remote keyboard "
         "cell_phone microwave oven toaster sink refrigerator book clock vase scissors teddy_bear hair_drier "
         "toothbrush").split()

class Yolo:
    def __init__(self, model, size=320, conf=0.25):
        so = ort.SessionOptions()
        so.intra_op_num_threads, so.inter_op_num_threads = 2, 1          # 11-13 cpu_bench: 기본은 4코어 + 대기 중에도 계속 돎(spin)
        so.add_session_config_entry("session.intra_op.allow_spinning", "0")   # → 2 Hz 인데 코어 0.77 개. 쉴 땐 진짜 쉬게
        self.sess = ort.InferenceSession(model, so, providers=["CPUExecutionProvider"])
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
