#!/usr/bin/env python3
"""[Pi] UGV02 피드백 단위 재기 도구 (ROS 없이 시리얼 직접). 브리지는 꺼두고 쓸 것 (포트 겹침).

  python3 scripts/odom_probe.py watch                    # 2초마다 L R odl odr gz v 출력 (Ctrl+C 끝)
  python3 scripts/odom_probe.py bias                     # 5초 정지 → 자이로 gz 평균(바이어스)
  python3 scripts/odom_probe.py straight --x 0.1 --t 5   # 직진 명령 T:13 X=0.1 을 5초 → 정지. 자로 실제 거리를 잰다
  python3 scripts/odom_probe.py spin --z 0.5 --t 6       # 제자리 회전 Z=0.5 를 6초 → 정지. 실제 각도를 잰다(바닥 표시)

각 실행 끝에 출력: 시간, odl/odr 변화량, ∫L dt / ∫R dt, ∫(gz − 바이어스) dt (raw 단위), 전압
→ 실제 거리·각도와 나누면 단위가 나온다. 기록: logs/probe_<모드>_<시각>.csv
안전: 명령은 0.1 s 마다 다시 보냄(펌웨어 3 s 타임아웃 대비), 끝·Ctrl+C·에러 때 항상 정지 명령.
"""
import argparse
import json
import os
import sys
import time
from datetime import datetime

import serial

PORT, BAUD = "/dev/ttyAMA0", 115200
REPO = os.path.realpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))


def cmd(ser, x, z):
    ser.write((json.dumps({"T": 13, "X": round(x, 3), "Z": round(z, 3)}) + "\n").encode())


def read_feedback(ser, buf):
    """버퍼에 쌓인 줄 중 T:1001 만 파싱해서 리스트로."""
    buf += ser.read(ser.in_waiting or 1).decode(errors="ignore")
    lines, buf = buf.split("\n")[:-1], buf.split("\n")[-1]
    out = []
    for line in lines:
        try:
            d = json.loads(line.strip())
            if d.get("T") == 1001:
                out.append(d)
        except ValueError:
            pass
    return out, buf


def run(mode, x, z, duration, bias):
    ser = serial.Serial(PORT, BAUD, timeout=0.05)
    ser.reset_input_buffer()
    stamp = f"{datetime.now():%Y%m%d_%H%M%S}"
    os.makedirs(os.path.join(REPO, "logs"), exist_ok=True)
    log_path = os.path.join(REPO, "logs", f"probe_{mode}_{stamp}.csv")
    log = open(log_path, "w")
    log.write("t,L,R,odl,odr,gx,gy,gz,v\n")

    rows, buf = [], ""
    t0 = last_cmd = last_print = time.monotonic()
    moving = mode in ("straight", "spin")
    try:
        while True:
            now = time.monotonic()
            t = now - t0
            if moving and now - last_cmd >= 0.1:
                cmd(ser, x, z); last_cmd = now
            fbs, buf = read_feedback(ser, buf)
            for d in fbs:
                row = (t, d.get("L", 0), d.get("R", 0), d.get("odl", 0), d.get("odr", 0),
                       d.get("gx", 0), d.get("gy", 0), d.get("gz", 0), d.get("v", 0))
                rows.append(row)
                log.write(",".join(str(v) for v in row) + "\n")
            if mode == "watch" and now - last_print >= 2.0 and rows:
                r = rows[-1]
                print(f"t={t:6.1f}  L={r[1]}  R={r[2]}  odl={r[3]}  odr={r[4]}  gz={r[7]}  v={r[8] / 100:.2f}V")
                last_print = now
            if mode != "watch" and t >= duration:
                break
    except KeyboardInterrupt:
        print("\n중단")
    finally:
        cmd(ser, 0, 0); time.sleep(0.05); cmd(ser, 0, 0)      # 정지 두 번
        # 멈춘 뒤 1초 더 읽어서 관성으로 굴러간 만큼도 포함
        end = time.monotonic() + 1.0
        while time.monotonic() < end:
            fbs, buf = read_feedback(ser, buf)
            for d in fbs:
                row = (time.monotonic() - t0, d.get("L", 0), d.get("R", 0), d.get("odl", 0), d.get("odr", 0),
                       d.get("gx", 0), d.get("gy", 0), d.get("gz", 0), d.get("v", 0))
                rows.append(row); log.write(",".join(str(v) for v in row) + "\n")
        ser.close(); log.close()

    if len(rows) < 2:
        print("피드백을 못 받음 (브리지가 포트를 잡고 있나? stty 115200?)"); return
    n = len(rows)
    dur = rows[-1][0] - rows[0][0]
    int_L = sum(rows[i][1] * (rows[i + 1][0] - rows[i][0]) for i in range(n - 1))
    int_R = sum(rows[i][2] * (rows[i + 1][0] - rows[i][0]) for i in range(n - 1))
    gz_mean = sum(r[7] for r in rows) / n
    int_gz = sum((rows[i][7] - bias) * (rows[i + 1][0] - rows[i][0]) for i in range(n - 1))
    print(f"\n=== {mode}  ({n}줄, {dur:.2f} s, 약 {n / dur:.1f} Hz) ===")
    print(f"odl: {rows[0][3]} → {rows[-1][3]}  (변화 {rows[-1][3] - rows[0][3]})")
    print(f"odr: {rows[0][4]} → {rows[-1][4]}  (변화 {rows[-1][4] - rows[0][4]})")
    print(f"∫L dt = {int_L:.4f}   ∫R dt = {int_R:.4f}   (|L|,|R| 최대 {max(abs(r[1]) for r in rows):.3f}, {max(abs(r[2]) for r in rows):.3f})")
    print(f"gz 평균 {gz_mean:.2f}   ∫(gz − 바이어스 {bias}) dt = {int_gz:.1f}  (raw·s)")
    print(f"전압 {rows[-1][8] / 100:.2f} V")
    print(f"기록: {log_path}")


def main():
    ap = argparse.ArgumentParser(description="UGV02 피드백 단위 재기")
    ap.add_argument("mode", choices=["watch", "bias", "straight", "spin"])
    ap.add_argument("--x", type=float, default=0.1, help="straight 속도 명령 X")
    ap.add_argument("--z", type=float, default=0.5, help="spin 회전 명령 Z")
    ap.add_argument("--t", type=float, default=5.0, help="명령 시간 [s] (bias 는 정지 측정 시간)")
    ap.add_argument("--bias", type=float, default=0.0, help="자이로 gz 바이어스 (bias 모드 결과)")
    a = ap.parse_args()
    if a.mode == "straight" and abs(a.x) > 0.3 or a.mode == "spin" and abs(a.z) > 1.5:
        sys.exit("처음엔 천천히: |x| ≤ 0.3, |z| ≤ 1.5")
    run(a.mode, a.x if a.mode == "straight" else 0.0, a.z if a.mode == "spin" else 0.0, a.t, a.bias)


if __name__ == "__main__":
    main()
