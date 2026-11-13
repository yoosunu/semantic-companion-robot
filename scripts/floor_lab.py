#!/usr/bin/env python3
"""floor_lab — 바닥 이상 감지 실험실 (11-13). "평소 바닥과 다르게 생긴 곳" = 뭔가 있다 (이름은 몰라도).

  고전 방법: 외관 기반 장애물 감지 (appearance-based obstacle detection, Ulrich & Nourbakhsh 2000 류)
    1. 배우기: 화면 아래 가운데(로봇 바로 앞, 방금 지나온 곳 = 확실히 바닥) 의 색을 Lab 색공간 평균·공분산으로 기억.
               영상마다 조금씩(EMA) 고쳐 감 — 조명 바뀌어도 따라감. 단, 그 칸 자체가 이상해 보이면 안 배움 (발닦개 위에 섰을 때)
    2. 찾기:   지평선(horizon) 아래 픽셀마다 기억한 바닥에서 얼마나 먼지 (마할라노비스 거리) → th 넘으면 "다름"
    3. 덩어리: 다른 픽셀을 이어서 넓이 min_area 넘는 것만 후보 상자 "?"
  YOLO 와 달리 COCO 에 없는 것도 됨 (발닦개·케이블·양말·멀티탭). 대신 그림자·햇빛·문턱 색 변화도 잡힘 → "피할 것" 이 아니라 "확인할 후보"
  가벼움: 320 폭에서 계산 (몇 ms). 고개는 앞·아래를 볼 때 의미 있음 (바닥이 화면 아래쪽에 보여야)

  쓰기 (Pi, 카메라 노드 필요 — floor_lab.sh 가 켜 줌)
    floor_lab.py --stream                 Mac 브라우저 http://<Pi IP>:8082
    floor_lab.py --stream --th 3.5        더 민감하게 (기본 4.0) / --horizon 0.5 (화면 위에서 이 비율 아래만, 기본 0.55)
    floor_lab.py --hz 5                    초당 5 번만 (기본: 영상마다)
    floor_lab.py --image 사진.jpg          사진 한 장 → ~/floor_lab/latest.jpg
  로그 1 s 마다: 후보 수·넓이, 처리 ms. 후보가 있으면 3 s 마다 그림 ~/floor_lab/seen_XX.jpg
"""
import os
import sys
import time

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from face_lab import frames, serve  # noqa: E402

OUT = os.path.expanduser("~/floor_lab")
W = 320                                   # 계산 폭


class Floor:
    def __init__(self, th=4.0, horizon=0.55, min_area=60, learn=0.05):
        self.th, self.horizon, self.min_area, self.learn = th, horizon, min_area, learn
        self.mu, self.icov, self.cov = None, None, None

    def ref_box(self, h, w):
        """기준 바닥 칸: 아래 가운데 (화면 맨 아래 8% 는 로봇 몸·글씨 가능성 → 뺌)"""
        return int(h * 0.80), int(h * 0.92), int(w * 0.35), int(w * 0.65)

    def step(self, bgr):
        """→ (다름 마스크 원본 크기, 후보 [(x,y,w,h,넓이px)] 원본 좌표, 기준칸 이상 여부)"""
        H, Wd = bgr.shape[:2]
        s = W / Wd
        small = cv2.resize(bgr, (W, int(H * s)))
        h, w = small.shape[:2]
        lab = cv2.cvtColor(cv2.GaussianBlur(small, (5, 5), 0), cv2.COLOR_BGR2Lab).astype(np.float32)
        y0, y1, x0, x1 = self.ref_box(h, w)
        ref = lab[y0:y1, x0:x1].reshape(-1, 3)
        mu, cov = ref.mean(0), np.cov(ref.T) + np.eye(3) * 4.0          # +4: 아주 고른 바닥이어도 너무 예민하지 않게
        odd_ref = False
        if self.mu is None:
            self.mu, self.cov = mu, cov
        else:
            d = mu - self.mu
            odd_ref = float(np.sqrt(d @ np.linalg.inv(self.cov) @ d)) > self.th   # 기준칸이 기억과 너무 다름 → 안 배움
            if not odd_ref:
                self.mu = (1 - self.learn) * self.mu + self.learn * mu
                self.cov = (1 - self.learn) * self.cov + self.learn * cov
        icov = np.linalg.inv(self.cov)
        d = lab.reshape(-1, 3) - self.mu
        dist = np.sqrt(np.einsum("ij,jk,ik->i", d, icov, d)).reshape(h, w)
        mask = (dist > self.th).astype(np.uint8)
        mask[: int(h * self.horizon)] = 0
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        n, lbl, stats, _ = cv2.connectedComponentsWithStats(mask)
        blobs = [(int(x / s), int(y / s), int(bw / s), int(bh / s), int(a / s / s))
                 for x, y, bw, bh, a in stats[1:] if a >= self.min_area]
        return cv2.resize(mask, (Wd, H), interpolation=cv2.INTER_NEAREST), blobs, odd_ref


def draw(bgr, mask, blobs, fl, ms, odd):
    img = bgr.copy()
    ov = img.copy()
    ov[mask > 0] = (255, 0, 255)
    img = cv2.addWeighted(ov, 0.45, img, 0.55, 0)
    H, Wd = img.shape[:2]
    y0, y1, x0, x1 = fl.ref_box(H, Wd)
    cv2.rectangle(img, (x0, y0), (x1, y1), (0, 0, 255) if odd else (0, 255, 0), 2)
    cv2.line(img, (0, int(H * fl.horizon)), (Wd, int(H * fl.horizon)), (255, 255, 0), 1)
    for x, y, bw, bh, a in blobs:
        cv2.rectangle(img, (x, y), (x + bw, y + bh), (0, 140, 255), 2)
        cv2.putText(img, f"? {a}", (x + 2, max(12, y - 4)), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 140, 255), 1)
    txt = f"floor {ms:.0f} ms  th {fl.th}" + ("  (ref odd: not learning)" if odd else "")
    cv2.putText(img, txt, (6, H - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    return img


def main():
    a = sys.argv[1:]
    arg = lambda k, d: type(d)(a[a.index(k) + 1]) if k in a else d
    fl = Floor(th=arg("--th", 4.0), horizon=arg("--horizon", 0.55), min_area=arg("--min-area", 60))
    os.makedirs(OUT, exist_ok=True)
    image = arg("--image", "")
    push = serve(8082) if "--stream" in a else None
    src = [cv2.imread(image)] if image else frames()
    t_log = t_save = t_last = 0.0
    k = 0
    hz = arg("--hz", 0.0)                        # 0 = 영상 올 때마다 / 2.0 = 초당 2 번만 (CPU 재기·실제 운용)
    for bgr in src:
        if hz and time.time() - t_last < 1.0 / hz:
            continue                              # 너무 이르면 이 영상은 건너뜀
        t_last = time.time()
        t0 = time.time()
        mask, blobs, odd = fl.step(bgr)
        ms = (time.time() - t0) * 1000
        img = draw(bgr, mask, blobs, fl, ms, odd)
        now = time.time()
        if push:
            push(img)
        if image or now - t_log >= 1.0:
            t_log = now
            big = sorted((b[4] for b in blobs), reverse=True)[:5]
            print(f"FLOOR {ms:.0f} ms | 후보 {len(blobs)}개 넓이 {big}{' | 기준칸 이상: 안 배움' if odd else ''}", flush=True)
            cv2.imwrite(os.path.join(OUT, "latest.jpg"), img)
        if not image and blobs and now - t_save >= 3.0:
            t_save = now
            cv2.imwrite(os.path.join(OUT, f"seen_{k % 30:02d}.jpg"), img)
            k += 1


if __name__ == "__main__":
    main()
