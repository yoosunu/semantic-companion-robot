#!/usr/bin/env python3
"""[Pi] ToF 실험실 — VL53L5CX (8×8, 앞판 정중앙, 보드 세로 장착) 검증만 (11-17). 아직 로봇 기능에는 안 씀.

  ROS 없이 I2C 로 직접 읽음 (브리지·LiDAR 꺼져 있어도 됨). 라이브러리: Pimoroni vl53l5cx-ctypes
    python3 -m venv --system-site-packages ~/tof-venv && ~/tof-venv/bin/pip install vl53l5cx-ctypes
  쓰기
    ~/tof-venv/bin/python tools/lab/tof_lab.py                    1 s 마다 8×8 표 (cm, 회전 반영) + 속도
    ~/tof-venv/bin/python tools/lab/tof_lab.py --raw              회전 안 한 원래 배열 (방향 확인용)
    ~/tof-venv/bin/python tools/lab/tof_lab.py --rot 3            90° 회전 횟수 (기본 1 = 반시계 90°). 방향 확인 뒤 고정
    ~/tof-venv/bin/python tools/lab/tof_lab.py --secs 10 --label wall50   10 s 기록 → 요약 (가운데 4칸 중앙값·흔들림, 칸별 최소)
  검증 순서 (handoff_hw_2026-11-17 2·3절)
    1. 방향: 센서 왼쪽 절반만 손으로 가림 → 표의 왼쪽이 가까워지면 회전 맞음 (위·아래도 같은 식)
    2. 가림: 앞에 아무것도 없는데 늘 가까운 칸 (스페이서·차체) → 요약의 "늘 가까움" 칸
    3. 거리: 벽에서 30·50·100 cm (줄자, 센서 면 기준) → 가운데 4칸 중앙값 vs 줄자
    4. 바닥: 아래쪽 줄이 바닥을 어디서부터 보나 (수평 장착이면 ~20~24 cm 앞부터 예상)
  표: 숫자 = cm, "--" = 무효 (target_status 5·9 아님 = 신뢰 안 함)
  기록: ~/scr_logs/tof_lab/<시각>_<label>.csv (시각, 칸 64개 mm, 상태 64개)
"""
import argparse
import csv
import os
import statistics as st
import time

import numpy as np

VALID = (5, 9)          # VL53L5CX target_status: 5 = 정상, 9 = 정상(이전 측정과 겹침 가능)
NEAR_MM = 60            # 이보다 가까우면 "가림 의심"


def grid(vals, status, rot, raw):
    d = np.array(vals[:64], dtype=float).reshape(8, 8)
    s = np.array(status[:64]).reshape(8, 8)
    d[~np.isin(s, VALID)] = np.nan
    return d if raw else np.rot90(d, rot)


def show(d, hz):
    print(f"--- {hz:.1f} Hz  (cm, 위 = 위쪽, 왼 = 로봇 왼쪽 — 방향 확인 전엔 가정)")
    for row in d:
        print(" ".join("  --" if np.isnan(v) else f"{v / 10:4.0f}" for v in row))


def main():
    a = argparse.ArgumentParser()
    a.add_argument("--rot", type=int, default=1)
    a.add_argument("--raw", action="store_true")
    a.add_argument("--hz", type=int, default=15)
    a.add_argument("--secs", type=float, default=0.0)
    a.add_argument("--label", default="test")
    o = a.parse_args()

    import vl53l5cx_ctypes as vl
    t0 = time.time()
    print("센서 켜는 중 (펌웨어 ~86 KB 올림, I2C 100 kHz 면 수 초)...", flush=True)
    s = vl.VL53L5CX()
    s.set_resolution(8 * 8)
    s.set_ranging_frequency_hz(o.hz)
    s.start_ranging()
    print(f"시작 {time.time() - t0:.1f} s", flush=True)

    rec, n, t_show, t_start = [], 0, 0.0, time.time()
    while True:
        if not s.data_ready():
            time.sleep(0.005)
            continue
        dt = s.get_data()
        dist = list(np.array(dt.distance_mm).flatten()[:64])
        stat = list(np.array(dt.target_status).flatten()[:64])
        now = time.time()
        n += 1
        rec.append((now, dist, stat))
        d = grid(dist, stat, o.rot, o.raw)
        if now - t_show >= 1.0:
            hz = n / (now - t_show) if t_show else 0.0
            t_show, n = now, 0
            show(d, hz)
        if o.secs and now - t_start >= o.secs:
            break

    s.stop_ranging()
    out = os.path.expanduser("~/scr_logs/tof_lab")
    os.makedirs(out, exist_ok=True)
    path = os.path.join(out, time.strftime("%Y%m%d_%H%M%S") + f"_{o.label}.csv")
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t"] + [f"d{i}" for i in range(64)] + [f"s{i}" for i in range(64)])
        for t, dd, ss in rec:
            w.writerow([round(t, 3)] + dd + ss)
    stack = np.array([grid(dd, ss, o.rot, o.raw) for _, dd, ss in rec])
    mid = stack[:, 3:5, 3:5].reshape(len(stack), -1)
    mid = [float(np.nanmedian(r)) for r in mid if not np.all(np.isnan(r))]
    near = np.nanmedian(stack, axis=0) < NEAR_MM
    valid = np.mean(~np.isnan(stack), axis=0)
    print(f"\n요약 {o.label}: {len(rec)} 장, {len(rec) / max(o.secs, 1e-6):.1f} Hz")
    if mid:
        print(f"  가운데 4칸 중앙값 {st.median(mid) / 10:.1f} cm, 흔들림(std) {st.pstdev(mid) / 10:.2f} cm")
    print(f"  유효 비율 평균 {valid.mean() * 100:.0f}%, 늘 가까움(<{NEAR_MM / 10:.0f} cm) 칸: {[tuple(map(int, x)) for x in np.argwhere(near)]}")
    print("  칸별 중앙값 (cm):")
    show(np.nanmedian(stack, axis=0), len(rec) / max(o.secs, 1e-6))
    print(f"  기록: {path}")


if __name__ == "__main__":
    main()
