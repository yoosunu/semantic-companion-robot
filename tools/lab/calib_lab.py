#!/usr/bin/env python3
"""[Pi] 카메라 렌즈 보정 (어안, 11-18). 체커보드를 여러 자세로 찍어 렌즈 모델 숫자를 구함 → 픽셀 ↔ 광선 방향.

  왜: 160° 어안은 핀홀(r = f·tanθ)로 못 담음 → OpenCV fisheye (Kannala-Brandt 계열) r = f·θ·(1 + k1θ² + k2θ⁴ + k3θ⁶ + k4θ⁸)
      구하는 것: K (fx, fy, cx, cy) + D (k1~k4). 바닥 감지·카메라 거리 (픽셀 → 바닥 좌표) 의 ① 단계.
  체커보드: tools/lab/checkerboard_9x6.png 를 아이패드에 꽉 차게 (안쪽 모서리 9×6). 칸 크기(mm)는 렌즈 값엔 필요 없음
  쓰기 (카메라 노드 필요: bash run/cam_run.sh 640 480 10 — **실제로 쓸 해상도로 보정**. 해상도 바꾸면 다시)
    calib_lab.py capture                 Mac 브라우저 http://<Pi IP>:8083 — 판이 보이면 초록, 자세가 충분히 다르면 자동 저장
    calib_lab.py capture 8               이번에 새로 8 장만 (기본 25). 이미 찍은 건 남음 → 빈 곳만 보충 (11-18: 전체 수로 세서 보충이 안 됐음)
    calib_lab.py solve                   저장한 사진으로 보정 → ~/calib/fisheye_<W>x<H>.json (+ 화면에 결과, 다시 찍기 오차 RMS px)
    calib_lab.py check                   펴진 영상 스트림 (8083): 직선(문틀·책장)이 곧게 보이면 OK
  확인 (11-18): 시뮬 판 20 장 (f 230, 잡음 0.2 px) → OpenCV 4.10 RMS 0.28 px, f 229.9 (맞음) / OpenCV 5.0 은 틀림
  찍는 법: 화면 가운데만 말고 **네 모서리·가장자리** 까지 (어안은 가장자리가 제일 휘고 제일 중요), 판을 기울여서도.
           흔들리면 흐려서 실패 → 잠깐 멈춰서. 아이패드 반사광 주의 (밝기 중간)
"""
import json
import os
import sys
import time

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from face_lab import frames, serve  # noqa: E402

COLS, ROWS = 9, 6                       # 안쪽 모서리
OUT = os.path.expanduser("~/calib")
N_WANT = 25


def find(gray):
    ok, c = cv2.findChessboardCorners(gray, (COLS, ROWS),
                                      cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE + cv2.CALIB_CB_FAST_CHECK)
    if ok:
        c = cv2.cornerSubPix(gray, c, (3, 3), (-1, -1), (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.01))
    return ok, c


def capture():
    os.makedirs(os.path.join(OUT, "imgs"), exist_ok=True)
    push = serve(8083)
    saved = []                           # (가운데 x, y, 크기) — 비슷한 자세는 또 안 찍음
    n = len([f for f in os.listdir(os.path.join(OUT, "imgs")) if f.endswith(".png")])
    want = int(sys.argv[2]) if len(sys.argv) > 2 else N_WANT
    n0 = n
    print(f"찍기 시작 (이미 {n} 장, 이번에 {want} 장). 판을 움직이며 모서리까지. Ctrl+C 로 끝", flush=True)
    last = 0.0
    for bgr in frames():
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        ok, c = find(gray)
        img = bgr.copy()
        H, W = gray.shape
        for x, y, s in saved:
            cv2.circle(img, (int(x), int(y)), 4, (255, 200, 0), -1)      # 찍은 자리 (파랑 점) — 빈 곳을 채우기
        if ok:
            cv2.drawChessboardCorners(img, (COLS, ROWS), c, ok)
            p = c.reshape(-1, 2)
            cx, cy = p.mean(0)
            size = float(np.linalg.norm(p.max(0) - p.min(0)))
            new = all(np.hypot(cx - x, cy - y) > 0.08 * W or abs(size - s) > 0.25 * s for x, y, s in saved)
            if new and time.time() - last > 1.0:
                n += 1
                last = time.time()
                cv2.imwrite(os.path.join(OUT, "imgs", f"{n:02d}.png"), bgr)
                saved.append((cx, cy, size))
                print(f"저장 {n:02d}: 가운데 ({cx:.0f},{cy:.0f}) 크기 {size:.0f}px", flush=True)
        cv2.putText(img, f"{'BOARD' if ok else 'no board'}  new {n - n0}/{want}  total {n}  {W}x{H}", (8, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0) if ok else (0, 0, 255), 2)
        push(img)
        if n - n0 >= want:
            print("충분함 → calib_lab.py solve", flush=True)
            break


def solve():
    d = os.path.join(OUT, "imgs")
    obj = np.zeros((1, COLS * ROWS, 3), np.float64)
    obj[0, :, :2] = np.mgrid[0:COLS, 0:ROWS].T.reshape(-1, 2)          # 칸 = 1 (렌즈 값엔 칸 크기 필요 없음)
    objs, imgs, names, size = [], [], [], None
    for f in sorted(os.listdir(d)):
        g = cv2.imread(os.path.join(d, f), cv2.IMREAD_GRAYSCALE)
        if g is None:
            continue
        ok, c = find(g)
        if ok:
            objs.append(obj)
            imgs.append(c.reshape(1, -1, 2).astype(np.float64))
            names.append(f)
            size = g.shape[::-1]
    print(f"판 찾음 {len(imgs)} 장 / {len(os.listdir(d))}")
    if len(imgs) < 8:
        print("8 장 넘게 필요")
        return
    K, D = np.zeros((3, 3)), np.zeros((4, 1))
    flags = 2 | 4 | 8          # fisheye RECOMPUTE_EXTRINSIC | CHECK_COND | FIX_SKEW (이름이 없는 판도 있어 숫자로)
    if cv2.__version__.startswith("5"):
        print(f"주의: OpenCV {cv2.__version__} — 5.0 의 fisheye.calibrate 는 시뮬에서 RMS 100+ 로 틀림 (4.10 은 0.28 px). 4.x 권장")
    while True:
        try:
            rms, K, D, rv, tv = cv2.fisheye.calibrate(objs, imgs, size, None, None, flags=flags,
                                                      criteria=(cv2.TERM_CRITERIA_COUNT + cv2.TERM_CRITERIA_EPS, 100, 1e-6))
            break
        except cv2.error as e:                                              # 나쁜 사진 하나가 계산을 망치면 빼고 다시
            msg = str(e)
            bad = [int(s) for s in msg.split() if s.isdigit()]
            if not bad or len(imgs) <= 8:
                print("보정 실패:", msg[:200])
                return
            i = bad[-1] if bad[-1] < len(imgs) else 0
            print(f"사진 {names[i]} 뺌 (계산 불안정)")
            for lst in (objs, imgs, names):
                lst.pop(i)
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    W, H = size
    res = {"model": "opencv_fisheye", "width": W, "height": H, "K": K.tolist(), "D": D.ravel().tolist(),
           "rms_px": rms, "images": len(imgs), "date": time.strftime("%Y-%m-%d")}
    path = os.path.join(OUT, f"fisheye_{W}x{H}.json")
    with open(path, "w") as f:
        json.dump(res, f, indent=1)
    # 가로 시야: 화면 왼쪽 끝 픽셀의 광선 각
    edge = cv2.fisheye.undistortPoints(np.array([[[0.0, cy], [W - 1.0, cy]]]), K, D)[0]
    fov_h = np.degrees(np.arctan(abs(edge[0, 0])) + np.arctan(abs(edge[1, 0])))
    print(f"\n결과 ({len(imgs)} 장, {W}x{H})")
    print(f"  fx {fx:.1f}  fy {fy:.1f}  cx {cx:.1f}  cy {cy:.1f}   (등거리 근사면 f ≈ {W / 2 / np.radians(80):.0f} = 160° 가정)")
    print(f"  D  {', '.join(f'{v:+.4f}' for v in D.ravel())}")
    print(f"  다시 찍기 오차 RMS {rms:.3f} px   (1 px 아래면 좋음)")
    print(f"  가로 시야 ≈ {fov_h:.0f}°")
    print(f"  저장: {path}")


def check():
    path = sorted(p for p in os.listdir(OUT) if p.startswith("fisheye_"))
    cal = json.load(open(os.path.join(OUT, path[-1])))
    K, D = np.array(cal["K"]), np.array(cal["D"])
    W, H = cal["width"], cal["height"]
    Kn = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(K, D, (W, H), np.eye(3), balance=0.3)
    m1, m2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), Kn, (W, H), cv2.CV_16SC2)
    push = serve(8083)
    print(f"{path[-1]} 로 펴기 — 왼쪽 원래, 오른쪽 펴짐 (8083)", flush=True)
    for bgr in frames():
        if bgr.shape[1] != W:
            print(f"해상도 다름 {bgr.shape[1]}x{bgr.shape[0]} vs 보정 {W}x{H}")
            continue
        und = cv2.remap(bgr, m1, m2, cv2.INTER_LINEAR)
        push(np.hstack([bgr, und]))


if __name__ == "__main__":
    {"capture": capture, "solve": solve, "check": check}.get(sys.argv[1] if len(sys.argv) > 1 else "", lambda: print(__doc__))()
