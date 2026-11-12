#!/usr/bin/env python3
"""face_lab — 얼굴 인식 실험실 (11-12). 결과를 그림으로 남겨서 눈으로 확인하며 배우기.

  얼굴 인식 = 두 단계
    ① 검출 YuNet  : "얼굴이 어디 있나" → 상자 + 점 5개(눈 2, 코, 입꼬리 2) + 점수      (YOLO bbox 와 같은 종류)
    ② 인식 SFace  : 점 5개로 얼굴을 똑바로 세워 112x112 로 자르고(정렬) → 숫자 128개(특징 벡터)
                    두 벡터의 코사인 유사도가 0.363 이상이면 같은 사람 (OpenCV 권장 기준)
    → 인식 결과는 상자가 아니라 "이름 + 유사도" 하나. 등록 = 내 얼굴 벡터를 여러 장 저장해 두는 것.

  쓰기 (Pi, 카메라 노드가 켜져 있어야 함 — face_lab.sh 가 켜 줌)
    face_lab.py enroll jasper     15 s 동안 고개를 천천히 좌우·위아래로 → 벡터 여러 장 저장 + 잘린 얼굴 모음 사진
    face_lab.py watch             1 s 마다 화면의 얼굴마다 이름·유사도 → 그림 ~/face_lab/latest.jpg (3 s 마다 기록도)
    face_lab.py forget jasper     등록 지우기
    face_lab.py watch --image 사진.png   카메라 없이 사진 한 장으로 (Mac/VM 시험용)

  개인정보: 얼굴 벡터는 생체정보. 본인 동의로만 등록, repo 밖 ~/.scr_faces/ (권한 600) 에만, forget 으로 지움.
"""
import glob
import json
import os
import sys
import time

import cv2
import numpy as np

HOME = os.path.expanduser("~")
DB = os.path.join(HOME, ".scr_faces")
OUT = os.path.join(HOME, "face_lab")
MODELS = os.path.join(HOME, "models")
DET_W = 320            # 검출은 320 폭으로 줄여서 (빠름). 자르기는 원본에서 (선명)
COS_OK = 0.363         # SFace 코사인 기준 (OpenCV 문서)
MIN_FACE = 28          # 320 폭 기준 얼굴 폭 px. 이보다 작으면 인식 안 함 (너무 흐림)


class Lab:
    def __init__(self, models=MODELS):
        self.det = cv2.FaceDetectorYN.create(os.path.join(models, "face_yunet.onnx"), "", (DET_W, DET_W), 0.6)
        self.rec = cv2.FaceRecognizerSF.create(os.path.join(models, "face_sface.onnx"), "")
        self.gallery = {}
        for p in glob.glob(os.path.join(DB, "*.npy")):
            self.gallery[os.path.basename(p)[:-4]] = np.load(p)

    def faces(self, bgr):
        """원본 좌표의 얼굴들 [(행 15개: x y w h, 점5개 x y, 점수), 320 기준 폭]"""
        h, w = bgr.shape[:2]
        s = DET_W / w
        small = cv2.resize(bgr, (DET_W, int(round(h * s))))
        self.det.setInputSize((small.shape[1], small.shape[0]))
        _, f = self.det.detect(small)
        out = []
        for r in ([] if f is None else f):
            full = r.copy()
            full[:14] /= s
            out.append((full, float(r[2])))
        return out

    def embed(self, bgr, face):
        crop = self.rec.alignCrop(bgr, face)           # 점 5개로 돌리고 잘라 112x112
        return crop, self.rec.feature(crop)

    def who(self, feat):
        """등록된 사람마다 가장 비슷한 한 장의 유사도 → (이름, 유사도). 기준 미만이면 이름 None"""
        best, name = -1.0, None
        for n, g in self.gallery.items():
            c = max(self.rec.match(feat, g[i:i + 1], cv2.FaceRecognizerSF_FR_COSINE) for i in range(len(g)))
            if c > best:
                best, name = c, n
        return (name if best >= COS_OK else None), best

    def draw(self, bgr, results):
        """상자·점 5개·이름·유사도 + 아래에 정렬된 얼굴 조각"""
        img = bgr.copy()
        sc = max(1, img.shape[1] // 320)
        for face, crop, name, cos, px in results:
            x, y, w, h = face[:4].astype(int)
            col = (0, 200, 0) if name else ((0, 0, 230) if crop is not None else (160, 160, 160))
            cv2.rectangle(img, (x, y), (x + w, y + h), col, sc)
            for i, c in zip(range(5), [(255, 0, 0), (0, 0, 255), (0, 255, 0), (255, 0, 255), (0, 255, 255)]):
                cv2.circle(img, (int(face[4 + 2 * i]), int(face[5 + 2 * i])), 2 * sc, c, -1)
            label = f"det {face[14]:.2f}"
            if crop is not None:
                label += f" | {name or '?'} {cos:.2f}"
            else:
                label += f" | small {px:.0f}px"
            cv2.putText(img, label, (x, max(12, y - 5)), cv2.FONT_HERSHEY_SIMPLEX, 0.4 * sc, col, sc)
        crops = [r[1] for r in results if r[1] is not None]
        if crops:
            strip = np.hstack(crops)
            if strip.shape[1] > img.shape[1]:
                strip = strip[:, : img.shape[1]]
            pad = np.zeros((strip.shape[0], img.shape[1], 3), np.uint8)
            pad[:, : strip.shape[1]] = strip
            cv2.putText(pad, "aligned 112x112 -> SFace", (strip.shape[1] + 6, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1)
            img = np.vstack([img, pad])
        return img

    def look(self, bgr):
        t0 = time.time()
        res = []
        for face, px in self.faces(bgr):
            if px < MIN_FACE:
                res.append((face, None, None, 0.0, px))
                continue
            crop, feat = self.embed(bgr, face)
            name, cos = self.who(feat)
            res.append((face, crop, name, cos, px))
        ms = (time.time() - t0) * 1000
        log = {"faces": len(res), "ms": round(ms),
               "who": [{"name": r[2], "cos": round(r[3], 3), "det": round(float(r[0][14]), 2), "px": round(r[4])} for r in res]}
        return res, log


def frames():
    """카메라 토픽에서 BGR 영상을 계속 (rclpy 는 여기서만 씀 → --image 시험은 ROS 없이)"""
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image
    rclpy.init()
    node = rclpy.create_node("face_lab")
    box = []
    node.create_subscription(Image, "camera/image_raw", lambda m: box.append(m), qos_profile_sensor_data)
    while rclpy.ok():
        box.clear()
        t0 = time.time()
        while not box and time.time() - t0 < 5:
            rclpy.spin_once(node, timeout_sec=0.1)
        if not box:
            print("영상 없음 (5 s) — 카메라 노드 켜졌나?")
            continue
        m = box[-1]
        a = np.frombuffer(bytes(m.data), np.uint8).reshape(m.height, m.step)[:, : m.width * 3].reshape(m.height, m.width, 3)
        yield a if m.encoding == "bgr8" else a[:, :, ::-1].copy()


def enroll(lab, name, secs=15.0):
    os.makedirs(DB, mode=0o700, exist_ok=True)
    os.makedirs(OUT, exist_ok=True)
    print(f"등록 {name}: {secs:.0f} s 동안 카메라를 보면서 고개를 천천히 좌우·위아래로 (한 명만 화면에)")
    feats, crops, last, end = [], [], 0.0, time.time() + secs
    for bgr in frames():
        now = time.time()
        if now > end or len(feats) >= 24:
            break
        if now - last < 0.4:
            continue
        fs = lab.faces(bgr)
        if len(fs) != 1 or fs[0][1] < MIN_FACE + 10 or fs[0][0][14] < 0.8:
            print(f"  건너뜀 (얼굴 {len(fs)}개{', 작음/흐림' if len(fs) == 1 else ''})")
            continue
        crop, feat = lab.embed(bgr, fs[0][0])
        feats.append(feat[0])
        crops.append(crop)
        last = now
        print(f"  {len(feats):2d}장 (폭 {fs[0][1]:.0f}px, 검출 {fs[0][0][14]:.2f})")
    if len(feats) < 5:
        print(f"{len(feats)}장뿐 → 저장 안 함. 밝은 곳에서 가까이 다시")
        return
    g = np.array(feats, np.float32)
    path = os.path.join(DB, name + ".npy")
    np.save(path, g)
    os.chmod(path, 0o600)
    n = g / np.linalg.norm(g, axis=1, keepdims=True)
    sim = n @ n.T
    off = sim[~np.eye(len(g), dtype=bool)]
    rows = [np.hstack(crops[i:i + 8] + [np.zeros_like(crops[0])] * (8 - len(crops[i:i + 8]))) for i in range(0, len(crops), 8)]
    cv2.imwrite(os.path.join(OUT, f"enroll_{name}.jpg"), np.vstack(rows))
    print(f"저장: {path} ({len(g)}장 × 128)  등록 사진끼리 유사도 평균 {off.mean():.2f} (최소 {off.min():.2f})")
    print(f"잘린 얼굴 모음: {OUT}/enroll_{name}.jpg")


def watch(lab, image=None):
    os.makedirs(OUT, exist_ok=True)
    print(f"등록된 사람: {', '.join(f'{k}({len(v)}장)' for k, v in lab.gallery.items()) or '없음 (전부 ? 로 나옴)'}")
    src = [cv2.imread(image)] if image else frames()
    k, t_save, t_log = 0, 0.0, 0.0
    for bgr in src:
        res, log = lab.look(bgr)
        now = time.time()
        if image or now - t_log >= 1.0:
            t_log = now
            print("FACE " + json.dumps(log, ensure_ascii=False))
            cv2.imwrite(os.path.join(OUT, "latest.jpg"), lab.draw(bgr, res))
        if not image and now - t_save >= 3.0 and res:
            t_save = now
            cv2.imwrite(os.path.join(OUT, f"watch_{k % 30:02d}.jpg"), lab.draw(bgr, res))
            k += 1
        if not image:
            time.sleep(0.5)


def main():
    a = sys.argv[1:]
    if not a or a[0] not in ("enroll", "watch", "forget"):
        print(__doc__)
        return
    if a[0] == "forget":
        for p in (os.path.join(DB, a[1] + ".npy"), os.path.join(OUT, f"enroll_{a[1]}.jpg")):
            if os.path.exists(p):
                os.remove(p)
                print("지움:", p)
        return
    models = MODELS
    if "--models" in a:
        models = a[a.index("--models") + 1]
    lab = Lab(models)
    if a[0] == "enroll":
        enroll(lab, a[1])
    else:
        watch(lab, a[a.index("--image") + 1] if "--image" in a else None)


if __name__ == "__main__":
    main()
