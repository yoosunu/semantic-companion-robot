"""floor_detect — 바닥 이상 감지 + 바닥 좌표 (순수 계산, 11-18). floor_perception 노드가 씀, tests/floor_sim.py 로 시험.

  tools/lab/floor_lab.py (11-13 실험) 의 Floor 를 노드용으로 옮김 (lab 은 그대로 둠). 바뀐 점:
    - 지평선을 화면 비율(0.55) 로 자르지 않고 **기하로**: ground.py 로 픽셀마다 바닥 거리 → max_range (1.5 m) 안 바닥만 봄
      (고개를 숙이든 들든 "바닥이 찍히는 곳" 이 자동으로 정해짐. 벽·가구 윗부분은 처음부터 안 봄)
    - 덩어리마다 **바닥에 닿은 아래 끝** (열마다 가장 아래 픽셀) → 바닥 (x, y) m = 장애물이 시작하는 곳
  방법 (고전, Ulrich & Nourbakhsh 2000 류 외관 기반 장애물 감지)
    1. 기준 바닥 = 로봇 바로 앞 칸의 Lab 색 평균·공분산 (EMA 로 천천히 따라감, 그 칸이 이상하면 안 배움)
    2. 픽셀마다 마할라노비스 거리 > th → "평소 바닥과 다름"
    3. 덩어리 (넓이 min_area 이상) → 후보. 그림자·햇빛·문턱도 잡힘 → "피할 것" 이 아니라 "후보" (반복되면 믿기: 다음 단계)
"""
import cv2
import numpy as np

import ground

W = 320                                   # 계산 폭


class FloorDetect:
    def __init__(self, K, D, size, th=4.0, min_area=60, learn=0.05, max_range=1.5, l_weight=1.0):
        self.th, self.min_area, self.learn, self.max_range = th, min_area, learn, max_range
        self.l_weight = l_weight                  # 밝기(L) 비중. 11-18 실물: 먼 바닥이 비스듬히 비쳐 더 밝음 → 통째로 "다름" → 낮추면 색(a,b) 위주
        self.K, self.D, (self.W0, self.H0) = K, D, size
        self.mu = self.cov = None
        self.pose = None
        self.valid = self.dist = None            # 작은 영상 크기: 바닥으로 볼 픽셀, 그 바닥 거리 (m)

    def set_pose(self, pan, tilt, h, w):
        """고개 자세마다 한 번: 작은 영상 픽셀 → 바닥 거리 표"""
        key = (round(pan, 1), round(tilt, 1), h, w)
        if key == self.pose:
            return
        self.pose = key
        s = self.W0 / w
        ys, xs = np.mgrid[0:h, 0:w]
        px = np.column_stack([(xs.ravel() + 0.5) * s, (ys.ravel() + 0.5) * s])
        fl = ground.pixels_to_floor(px, self.K, self.D, pan, tilt)
        d = np.hypot(fl[:, 0], fl[:, 1]).reshape(h, w)
        self.dist = d
        self.valid = np.isfinite(d) & (d < self.max_range)

    def ref_box(self, h, w):
        """기준 바닥 칸: 화면 아래 가운데 (맨 아래 8% 는 로봇 몸·글씨 가능성 → 뺌)"""
        return int(h * 0.80), int(h * 0.92), int(w * 0.35), int(w * 0.65)

    def step(self, bgr, pan, tilt):
        """→ (blobs, odd_ref, mask 작은 크기)
           blobs: [{"box": (x,y,w,h) 원본 px, "area": px, "near": m, "pts": N×2 바닥 (x,y) m (아래 끝)}] 가까운 순"""
        H0, W0 = bgr.shape[:2]
        s = W / W0
        small = cv2.resize(bgr, (W, int(H0 * s)))
        h, w = small.shape[:2]
        self.set_pose(pan, tilt, h, w)
        lab = cv2.cvtColor(cv2.GaussianBlur(small, (5, 5), 0), cv2.COLOR_BGR2Lab).astype(np.float32)
        lab[:, :, 0] *= self.l_weight
        y0, y1, x0, x1 = self.ref_box(h, w)
        odd = False
        if self.valid[y0:y1, x0:x1].mean() > 0.9:                      # 기준 칸이 정말 바닥일 때만 배움
            ref = lab[y0:y1, x0:x1].reshape(-1, 3)
            mu, cov = ref.mean(0), np.cov(ref.T) + np.eye(3) * 4.0
            if self.mu is None:
                self.mu, self.cov = mu, cov
            else:
                d = mu - self.mu
                odd = float(np.sqrt(d @ np.linalg.inv(self.cov) @ d)) > self.th
                if not odd:
                    self.mu = (1 - self.learn) * self.mu + self.learn * mu
                    self.cov = (1 - self.learn) * self.cov + self.learn * cov
        if self.mu is None:
            return [], odd, np.zeros((h, w), np.uint8)
        d = lab.reshape(-1, 3) - self.mu
        dist = np.sqrt(np.einsum("ij,jk,ik->i", d, np.linalg.inv(self.cov), d)).reshape(h, w)
        mask = ((dist > self.th) & self.valid).astype(np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        n, lbl, stats, _ = cv2.connectedComponentsWithStats(mask)
        blobs = []
        for i in range(1, n):
            x, y, bw, bh, a = stats[i]
            if a < self.min_area:
                continue
            ys, xs = np.nonzero(lbl == i)
            bottom = {}
            for xx, yy in zip(xs, ys):                                   # 열마다 가장 아래 = 바닥에 닿은 곳
                if yy > bottom.get(xx, -1):
                    bottom[xx] = yy
            cols = sorted(bottom)[::2]
            px = np.array([[(c + 0.5) / s, (bottom[c] + 0.5) / s] for c in cols])
            pts = ground.pixels_to_floor(px, self.K, self.D, pan, tilt)
            pts = pts[np.isfinite(pts).all(1)]
            if not len(pts):
                continue
            blobs.append({"box": (int(x / s), int(y / s), int(bw / s), int(bh / s)), "area": int(a / s / s),
                          "near": float(np.min(np.hypot(pts[:, 0], pts[:, 1]))), "pts": pts})
        blobs.sort(key=lambda b: b["near"])
        return blobs, odd, mask


def draw(bgr, fd, blobs, mask, odd, ms):
    img = bgr.copy()
    H0, W0 = img.shape[:2]
    big = cv2.resize(mask, (W0, H0), interpolation=cv2.INTER_NEAREST)
    ov = img.copy()
    ov[big > 0] = (255, 0, 255)
    img = cv2.addWeighted(ov, 0.45, img, 0.55, 0)
    edge = cv2.resize(fd.valid.astype(np.uint8), (W0, H0), interpolation=cv2.INTER_NEAREST)
    cs, _ = cv2.findContours(edge, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(img, cs, -1, (255, 255, 0), 1)                     # 하늘색 = 보는 바닥 (max_range 안)
    for b in blobs:
        x, y, bw, bh = b["box"]
        cv2.rectangle(img, (x, y), (x + bw, y + bh), (0, 140, 255), 2)
        cv2.putText(img, f"{b['near'] * 100:.0f}cm", (x + 2, max(12, y - 4)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 140, 255), 2)
    cv2.putText(img, f"floor {ms:.0f} ms  {len(blobs)} blobs" + ("  (ref odd)" if odd else ""), (6, H0 - 8),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    return img
