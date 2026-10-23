#!/usr/bin/env python3
"""맵 전체 순회: 바깥 고리 한 바퀴 → 기둥 사이 통로 → 출발점. 매번 같은 경로라서 설정 비교에 쓴다.

  robot tour            (robot 이 --mode map|slam 을 붙여서 실행)

채점 (Gazebo 정답 위치 대비):
  - 경로 오차 ATE: 1초마다 "로봇이 믿는 위치(TF map→base_footprint)"와 "실제 위치(Gazebo)"의 차이 → RMSE·최대
    두 좌표계를 "전체 궤적이 가장 잘 겹치게" 회전·이동으로 맞춘 뒤 비교 (SLAM 평가 표준 방식).
    맞출 때 필요했던 회전각 = 지도 전체가 실제 대비 얼마나 돌아가 있나 (따로 출력)
    (10-23: 출발 시점 한 점으로만 맞추면, 시작 순간의 방향 오차 몇 도가 먼 곳에서 수십 cm로 부풀어 보였음 → 표준 방식으로 교체)
  - 최종 오차: 출발 시점을 기준으로 맞췄을 때, 출발점에 돌아온 순간의 위치·방향 차이 = 한 바퀴 동안 쌓인 표류
기록: logs/tour_<시각>.csv (궤적), logs/tour_<시각>.txt (요약 + 그때 설정값)
SLAM 모드: 끝나면 지도 저장 logs/maps/tour_<시각>.{pgm,yaml}
"""
import argparse
import math
import os
import re
import subprocess
import threading
import time
from datetime import datetime

import rclpy
from rclpy.signals import SignalHandlerOptions

from shuttle import Shuttle, wrap  # 같은 폴더: 목표 보내기·취소·위치 읽기는 왕복 도구와 공유

REPO = os.path.realpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

# map 좌표 (출발점 = 0,0). 기둥 중심 x ∈ {0.9, 2.0, 3.1}, y ∈ {-0.6, 0.5, 1.6}, 벽 대략 x -0.95~4.7, y -2.1~3.1
OUTER = [(2.0, -1.4), (3.49, -0.99), (3.9, 0.5), (3.49, 1.99),    # 기둥 무리 바깥 고리 (중심 2.0, 0.5 에서 약 1.9~2.1 m)
         (2.0, 2.4), (0.51, 1.99), (0.1, 0.5), (0.51, -0.99)]
INNER = [(1.45, -0.05), (3.65, -0.05), (3.65, 1.05), (0.35, 1.05)]  # 기둥 사이 통로 두 줄
FINISH = (0.0, 0.0, 0.0)                                             # 출발점 (루프 클로저 기회)
MAP_OFFSET = (2.0, 0.5)  # map = Gazebo 월드 + (2.0, 0.5) — 로봇 생성 위치. 정렬은 출발 시점으로 다시 함


def route():
    pts = OUTER + INNER
    poses = []
    for i, (x, y) in enumerate(pts):
        nx, ny = pts[i + 1] if i + 1 < len(pts) else FINISH[:2]
        poses.append((x, y, math.atan2(ny - y, nx - x)))   # 다음 지점을 바라보게 → 제자리 회전 줄이기
    return poses + [FINISH]


def gazebo_truth():
    """Gazebo 실제 위치 (map 좌표 근사, x y yaw). 실패하면 None."""
    try:
        out = subprocess.run(["gz", "model", "-m", "burger", "-p"], capture_output=True, text=True, timeout=3).stdout
        rows = [l for l in out.splitlines() if re.match(r"\s*\[", l)]
        num = r"-?\d+\.\d+(?:e-?\d+)?"
        xyz = [float(v) for v in re.findall(num, rows[0])]
        rpy = [float(v) for v in re.findall(num, rows[1])]
        return xyz[0] + MAP_OFFSET[0], xyz[1] + MAP_OFFSET[1], rpy[2]
    except Exception:
        return None


class Recorder(threading.Thread):
    """1초마다 (시간, 실제 위치, 로봇이 믿는 위치) 기록."""

    def __init__(self, node, period=1.0):
        super().__init__(daemon=True)
        self.node, self.period, self.samples, self.running = node, period, [], True
        self.t0 = time.monotonic()

    def run(self):
        while self.running:
            truth, est = gazebo_truth(), self.node.robot_pose()
            if truth and est:
                self.samples.append((time.monotonic() - self.t0, truth, est))
            time.sleep(self.period)


def align(samples):
    """첫 샘플에서 '실제 → 믿는 좌표계' 2D 변환을 구해 모든 실제 위치를 옮긴다."""
    _, (tx, ty, tth), (ex, ey, eth) = samples[0]
    dth = wrap(eth - tth)
    c, s = math.cos(dth), math.sin(dth)
    ox, oy = ex - (c * tx - s * ty), ey - (s * tx + c * ty)
    out = []
    for t, (x, y, th), est in samples:
        out.append((t, (c * x - s * y + ox, s * x + c * y + oy, wrap(th + dth)), est))
    return out


def best_fit(samples):
    """전체 궤적이 가장 잘 겹치는 2D 회전·이동을 찾아 ATE 계산 (최소제곱, Horn/Umeyama 2D판).
    반환: (회전각 deg, 점별 오차 리스트)"""
    T = [tr[:2] for _, tr, _ in samples]
    E = [es[:2] for _, _, es in samples]
    n = len(T)
    tx, ty = sum(p[0] for p in T) / n, sum(p[1] for p in T) / n
    ex, ey = sum(p[0] for p in E) / n, sum(p[1] for p in E) / n
    sxx = sxy = 0.0
    for (a, b), (c, d) in zip(T, E):
        a, b, c, d = a - tx, b - ty, c - ex, d - ey
        sxx += a * c + b * d
        sxy += a * d - b * c
    th = math.atan2(sxy, sxx)
    C, S = math.cos(th), math.sin(th)
    errs = [math.hypot(C * (a - tx) - S * (b - ty) + ex - c, S * (a - tx) + C * (b - ty) + ey - d)
            for (a, b), (c, d) in zip(T, E)]
    return math.degrees(th), errs


def my_settings(path):
    """설정값 파일에서 주석이 아닌 줄만 (어떤 설정으로 돌렸는지 요약에 남기기)."""
    try:
        lines = [l.split("#")[0].strip() for l in open(path)]
        return [l for l in lines if l and not l.endswith(":")] or ["(변경 없음)"]
    except OSError:
        return ["(파일 없음)"]


def save_map(stamp, prefix="tour"):
    out_dir = os.path.join(REPO, "logs", "maps"); os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, f"{prefix}_{stamp}")
    r = subprocess.run(["ros2", "run", "nav2_map_server", "map_saver_cli", "-f", path, "--ros-args",
                        "-p", "use_sim_time:=true", "-p", "map_subscribe_transient_local:=false",
                        "-p", "save_map_timeout:=10.0"], capture_output=True, text=True, timeout=40)
    return path if r.returncode == 0 else None


def read_pgm(path):
    """(width, height, bytes). P5 형식만."""
    data = open(path, "rb").read()
    parts = data.split(maxsplit=4)
    w, h = int(parts[1]), int(parts[2])
    return w, h, parts[4][-w * h:]


def read_map(yaml_path):
    """map_saver 결과 → (w, h, pixels, origin_x, origin_y, resolution)."""
    meta = {}
    for line in open(yaml_path):
        if ":" in line:
            k, v = line.split(":", 1); meta[k.strip()] = v.strip()
    ox, oy = [float(v) for v in meta["origin"].strip("[]").split(",")[:2]]
    w, h, px = read_pgm(os.path.join(os.path.dirname(yaml_path), meta["image"]))
    return w, h, px, ox, oy, float(meta["resolution"])


def coverage(map_yaml, ref_yaml=os.path.join(REPO, "maps", "tb3_world_cartographer.yaml")):
    """기준 지도의 빈 칸 중, 새 지도에서도 '빈 칸으로 확인된' 비율 (%). 두 지도 모두 원점 = 로봇 출발점."""
    rw, rh, rpx, rox, roy, rres = read_map(ref_yaml)
    w, h, px, ox, oy, res = read_map(map_yaml)
    total = seen = 0
    for j in range(rh):
        for i in range(rw):
            if rpx[j * rw + i] < 250:          # 기준 지도에서 빈 칸(254)만
                continue
            x = rox + (i + 0.5) * rres
            y = roy + (rh - 1 - j + 0.5) * rres
            total += 1
            ii, jj = int((x - ox) / res), h - 1 - int((y - oy) / res)
            if 0 <= ii < w and 0 <= jj < h and px[jj * w + ii] >= 250:
                seen += 1
    return 100.0 * seen / total if total else float("nan")


def distance(samples):
    """실제 이동 거리 (1초 간격 샘플이라 약간 짧게 나옴)."""
    return sum(math.hypot(b[1][0] - a[1][0], b[1][1] - a[1][1]) for a, b in zip(samples, samples[1:]))


def main():
    ap = argparse.ArgumentParser(description="맵 전체 순회 + 정답 대비 채점")
    ap.add_argument("--mode", default="map", help="map | slam (robot 이 자동으로 붙임)")
    args = ap.parse_args()
    if gazebo_truth() is None:
        print("Gazebo 실제 위치를 못 읽음 (gz model). 시뮬이 켜져 있나요? → robot status"); return

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Shuttle()
    stamp = f"{datetime.now():%Y%m%d_%H%M%S}"
    rec = None
    results = []
    try:
        print("준비 확인 중...")
        start_wait = time.monotonic()
        while not (node.nav.server_is_ready() and node.nav2_ready() and node.robot_pose() is not None):
            node.settle(1.0)
            print(f"  아직 ({time.monotonic() - start_wait:.0f} s)... 오래 걸리면 robot status")
        poses = route()
        print(f"모드: {args.mode}  |  지점 {len(poses)}개 (바깥 고리 {len(OUTER)} → 통로 {len(INNER)} → 출발점)\n")

        rec = Recorder(node); rec.start()
        t_start = time.monotonic()
        for i, pose in enumerate(poses, 1):
            t = time.monotonic()
            result = node.go(pose)
            results.append(result)
            print(f"  [{i:2d}/{len(poses)}] ({pose[0]:5.2f}, {pose[1]:5.2f})  {result:9s} {time.monotonic() - t:5.1f} s")
            if result == "REJECTED":
                print("  → Nav2가 거절. 멈춤 → robot status"); break
        total = time.monotonic() - t_start
    except KeyboardInterrupt:
        node.cancel(); total = time.monotonic() - t_start if rec else 0.0
        print("\n중단됨 — 지금까지 기록으로 채점")
    finally:
        if rec:
            node.settle(1.0); rec.running = False; rec.join(timeout=3)

    try:
        if not rec or len(rec.samples) < 2:
            print("기록이 부족해서 채점 못 함"); return
        rot_deg, errs = best_fit(rec.samples)                      # ATE (표준: 전체 궤적 기준 정렬)
        rmse = math.sqrt(sum(e * e for e in errs) / len(errs))
        samples = align(rec.samples)                                # 표류 (출발 시점 기준 정렬)
        _, tr_end, es_end = samples[-1]
        drift_end = math.hypot(tr_end[0] - es_end[0], tr_end[1] - es_end[1])
        yaw_end = math.degrees(abs(wrap(tr_end[2] - es_end[2])))
        ok = results.count("SUCCEEDED")

        summary = [
            f"=== tour {stamp}  mode={args.mode} ===",
            f"지점 성공 {ok}/{len(results)}, 총 {total:.1f} s, 샘플 {len(samples)}개",
            f"경로 오차 ATE: RMSE {rmse * 100:.1f} cm, 최대 {max(errs) * 100:.1f} cm  (지도 전체 회전 {rot_deg:+.1f}°)",
            f"최종 표류 (출발 기준, 복귀 시점): 위치 {drift_end * 100:.1f} cm, 방향 {yaw_end:.1f}°",
            "설정 my_slam.yaml: " + " | ".join(my_settings(os.path.join(REPO, "config", "my_slam.yaml"))),
            "설정 my_nav2.yaml: " + " | ".join(my_settings(os.path.join(REPO, "config", "my_nav2.yaml"))),
        ]
        summary.insert(2, f"이동 거리 약 {distance(samples):.1f} m")
        if args.mode == "slam":
            print("\n지도 저장 중...")
            path = save_map(stamp)
            if path:
                summary.append(f"지도 완성도: 기준 지도 빈 공간의 {coverage(path + '.yaml'):.1f}%")
                summary.append(f"지도: {path}.pgm")
            else:
                summary.append("지도 저장 실패 (robot status, nav2 창 확인)")
        print("\n" + "\n".join(summary))

        log = os.path.join(REPO, "logs", f"tour_{stamp}")
        with open(log + ".txt", "w") as f:
            f.write("\n".join(summary) + "\n")
        with open(log + ".csv", "w") as f:
            f.write("t_s,true_x,true_y,true_yaw,est_x,est_y,est_yaw,err_m\n")  # true = 출발 기준 정렬, err = 표준(전체) 정렬 오차
            for (t, tr, es), e in zip(samples, errs):
                f.write(f"{t:.1f},{tr[0]:.3f},{tr[1]:.3f},{tr[2]:.3f},{es[0]:.3f},{es[1]:.3f},{es[2]:.3f},{e:.3f}\n")
        print(f"기록: {log}.txt / .csv")
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
