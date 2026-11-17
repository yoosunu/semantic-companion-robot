"""ground — 카메라 픽셀 ↔ 바닥 좌표 (순수 계산, 11-18). 바닥 감지·카메라 거리·YOLO 상자 아래 끝 거리가 다 이걸 씀.

  레이캐스트 세 단계 (SimCheong 의 ScreenPointToRay + 지형 레이캐스트와 같은 일, 렌즈부터 직접)
    ① 렌즈 (calib_lab 으로 보정한 OpenCV fisheye K·D): 픽셀 → 카메라 기준 광선 방향
    ② 카메라 자세 (실측 + 팬·틸트 각): 카메라 광선 → 로봇(base_footprint) 기준 광선
    ③ 바닥 평면 z = 0 과 만나는 점 → (x 앞, y 왼쪽) m
  좌표: 로봇 = base_footprint (차체 중심 바로 아래 바닥, x 앞, y 왼, z 위 — URDF 와 같음). 카메라 = OpenCV (x 오른쪽, y 아래, z 앞)
  팬 + = 오른쪽, 틸트 + = 위 (/pantilt/cmd 와 같음)

  실측 (11-18 Jasper, 줄자 ±0.5 cm)
    TILT_H  바닥 → 틸트 축 24.5 cm
    LENS_FWD, LENS_UP  틸트 축 → 렌즈 (고개 0,0 일 때) 앞 6.5 cm, 위 0.5 cm
    LASER_X  차체 중심 → LiDAR 중심 앞 7.8 cm (차체 앞 끝 → LiDAR 4.8 cm, 차체 길이 25.2). URDF laser_x 와 같음
    PAN_X, PAN_Y  팬 축 = LiDAR 중심에서 뒤 7 cm → 차체 중심 기준 +0.8 cm, 좌우 0 (틸트 축은 팬 축 바로 위라고 가정)
    TILT_OFFSET  틸트 명령 0° 일 때 카메라 실제 시선 +2.3° (위). 서보 0점은 손 보정 (handoff_pantilt 11-11) → 0점 재저장
                 대신 여기서만 보정 (눈맞춤 등 다른 동작은 지금 0점으로 맞춰져 있어서)
  검증 (11-18, 건전지 3개 = 앞 끝에서 17·37·87 cm, 고개 −20 + 0 에서 바닥 닿은 점):
    차체 중심 기준 정답 29.6 / 49.6 / 99.6 / 49.6 → 모델 30.0 / 49.9 / 99.2 / 51.2 (최대 1.6 cm, 캡처에서 읽은 픽셀)
    처음엔 "LiDAR = 차체 중심 (앞 끝에서 13 cm)" 가정 → 0점 7° 로 50 cm 한 점만 맞고 30·100 은 −4·+39 cm.
    LiDAR 위치를 미지수로 풀어 4.8~6 cm 예측 → 줄자 4.8 cm 로 확인 → URDF laser_x 도 고침
"""
import json
import math
import os

import numpy as np

TILT_H = 0.245
LENS_FWD, LENS_UP = 0.065, 0.005
LASER_X = 0.078
PAN_X, PAN_Y = LASER_X - 0.07, 0.0
TILT_OFFSET = 2.3
CALIB = os.path.expanduser("~/calib/fisheye_640x480.json")


def load_calib(path=CALIB):
    c = json.load(open(path))
    return np.array(c["K"], float), np.array(c["D"], float).reshape(4, 1), (c["width"], c["height"])


def cam_pose(pan_deg, tilt_deg):
    """pan_deg, tilt_deg = /pantilt/cmd 명령값 → (R, t): 카메라 좌표(OpenCV) 의 벡터를 로봇 좌표로 R @ v, 렌즈 위치 t (m)"""
    p, q = math.radians(-pan_deg), math.radians(tilt_deg + TILT_OFFSET)   # 팬 + = 오른쪽 → z 축 음의 회전. 틸트 = 명령 + 0점 보정
    Rz = np.array([[math.cos(p), -math.sin(p), 0], [math.sin(p), math.cos(p), 0], [0, 0, 1]])
    Ry = np.array([[math.cos(q), 0, -math.sin(q)], [0, 1, 0], [math.sin(q), 0, math.cos(q)]])   # 틸트 + = 앞이 위로
    C = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]], float)     # 카메라(x 오른, y 아래, z 앞) → 머리(x 앞, y 왼, z 위)
    R = Rz @ Ry @ C
    t = np.array([PAN_X, PAN_Y, TILT_H]) + Rz @ Ry @ np.array([LENS_FWD, 0.0, LENS_UP])
    return R, t


def pixels_to_floor(px, K, D, pan_deg, tilt_deg):
    """px: N×2 픽셀 → N×2 바닥 (x, y) m. 바닥에 안 닿는 광선(수평 위) 은 nan"""
    import cv2
    px = np.asarray(px, float).reshape(-1, 1, 2)
    und = cv2.fisheye.undistortPoints(px, K, D).reshape(-1, 2)          # 정규 좌표 (x/z, y/z)
    rays = np.column_stack([und, np.ones(len(und))])
    R, t = cam_pose(pan_deg, tilt_deg)
    w = rays @ R.T
    out = np.full((len(w), 2), np.nan)
    down = w[:, 2] < -1e-6
    s = -t[2] / w[down, 2]
    out[down] = t[:2] + w[down, :2] * s[:, None]
    return out


def floor_to_pixels(pts, K, D, pan_deg, tilt_deg):
    """pts: N×2 바닥 (x, y) m → N×2 픽셀 (카메라 뒤면 nan)"""
    import cv2
    pts = np.asarray(pts, float).reshape(-1, 2)
    R, t = cam_pose(pan_deg, tilt_deg)
    P = np.column_stack([pts, np.zeros(len(pts))]) - t
    cam = P @ R                                                          # 로봇 → 카메라 (R 의 전치)
    out = np.full((len(cam), 2), np.nan)
    front = cam[:, 2] > 0.02
    if front.any():
        img, _ = cv2.fisheye.projectPoints(cam[front].reshape(-1, 1, 3), np.zeros(3), np.zeros(3), K, D)
        out[front] = img.reshape(-1, 2)
    return out
