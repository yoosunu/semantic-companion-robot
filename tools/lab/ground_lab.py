#!/usr/bin/env python3
"""[Pi] 바닥 격자 겹쳐 보기 (11-18) — 렌즈 보정 + 카메라 실측이 맞나 눈으로 확인. nodes/perception/ground.py 를 씀.

  준비: 로봇 켜기 (고개 명령용) + 카메라 640 480. 바닥에 줄자를 로봇 앞으로 쭉 펴고 30·50·100 cm 에 테이프 표시
        (거리 기준 = LiDAR 중심 바로 아래 바닥. LiDAR 중심 → 차체 앞 끝 거리를 빼고 재도 됨)
  쓰기
    ground_lab.py                      고개 (0, −20) 로 돌리고 격자 (8084)
    ground_lab.py --pan 0 --tilt -30   다른 자세 — 자세가 바뀌어도 격자가 바닥 같은 자리에 붙어 있어야 맞음
    ground_lab.py --no-move            고개 안 움직임 (명령한 각이 지금 각이라고 믿고)
  그림: 앞 10 cm 마다 가로선 (30·50·100 cm 굵게, 숫자), 좌우 10 cm 마다 세로선. 노랑 = 화면 아래 가운데 픽셀의 바닥 거리
"""
import argparse
import os
import sys
import time

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, "..", "..", "nodes", "perception"))
import ground  # noqa: E402
from face_lab import frames, serve  # noqa: E402


def head(pan, tilt):
    import rclpy
    from std_msgs.msg import Float32MultiArray
    if not rclpy.ok():
        rclpy.init()
    n = rclpy.create_node("ground_lab_head")
    pub = n.create_publisher(Float32MultiArray, "pantilt/cmd", 10)
    for _ in range(5):
        pub.publish(Float32MultiArray(data=[float(pan), float(tilt), 300.0, 10.0]))
        rclpy.spin_once(n, timeout_sec=0.1)
    time.sleep(2.0)
    n.destroy_node()
    rclpy.shutdown()                                   # frames() 가 다시 rclpy.init() 함 (두 번 init 은 에러)


def draw(img, K, D, pan, tilt):
    H, W = img.shape[:2]
    for d in range(10, 301, 10):                                         # 앞 거리 선
        ys = np.linspace(-1.5, 1.5, 61)
        px = ground.floor_to_pixels(np.column_stack([np.full_like(ys, d / 100), ys]), K, D, pan, tilt)
        bold = d in (30, 50, 100, 200)
        pts = [tuple(map(int, p)) for p in px if np.isfinite(p).all() and -50 < p[0] < W + 50 and -50 < p[1] < H + 50]
        if len(pts) > 1:
            cv2.polylines(img, [np.array(pts)], False, (0, 255, 255) if bold else (0, 160, 0), 2 if bold else 1)
            if bold:
                c = ground.floor_to_pixels([[d / 100, 0.0]], K, D, pan, tilt)[0]
                if np.isfinite(c).all():
                    cv2.putText(img, f"{d}", (int(c[0]) + 4, int(c[1]) - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
    for yv in range(-50, 51, 10):                                        # 좌우 선
        xs = np.linspace(0.1, 3.0, 60)
        px = ground.floor_to_pixels(np.column_stack([xs, np.full_like(xs, yv / 100)]), K, D, pan, tilt)
        pts = [tuple(map(int, p)) for p in px if np.isfinite(p).all() and -50 < p[0] < W + 50 and -50 < p[1] < H + 50]
        if len(pts) > 1:
            cv2.polylines(img, [np.array(pts)], False, (255, 120, 0) if yv == 0 else (0, 120, 0), 1)
    b = ground.pixels_to_floor([[W / 2, H - 5]], K, D, pan, tilt)[0]
    cv2.putText(img, f"pan {pan:+.0f} tilt {tilt:+.0f} | bottom-center pixel -> floor {b[0] * 100:.0f} cm", (8, 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
    return img


def main():
    a = argparse.ArgumentParser()
    a.add_argument("--pan", type=float, default=0.0)
    a.add_argument("--tilt", type=float, default=-20.0)
    a.add_argument("--no-move", action="store_true")
    o = a.parse_args()
    K, D, (W, H) = ground.load_calib()
    if not o.no_move:
        head(o.pan, o.tilt)
    push = serve(8084)
    print(f"격자 (8084): 고개 ({o.pan:+.0f}, {o.tilt:+.0f}), 보정 {W}x{H}", flush=True)
    for bgr in frames():
        if bgr.shape[1] != W:
            print(f"해상도 {bgr.shape[1]}x{bgr.shape[0]} ≠ 보정 {W}x{H}")
            continue
        push(draw(bgr.copy(), K, D, o.pan, o.tilt))


if __name__ == "__main__":
    main()
