#!/usr/bin/env python3
"""[Pi] IMU 실험실 — "민감한 로봇" (11-06 Jasper: 기어다니니까 충돌·턱·카펫·건드림을 IMU 로 느낄 수 있다).

브리지(v6)가 내는 /imu 를 기록하고, 시험 하나가 끝나면 바로 숫자를 보여준다. 로봇만 켜고 (robot.launch, Nav2 없이) 쓴다.

  python3 scripts/imu_lab.py still --label rest --secs 8            가만히 (기준값). 이때 손으로 톡 치기·밀기·들기도 시험
  python3 scripts/imu_lab.py drive --label floor --secs 4            앞으로 0.08 m/s 4초 (평평한 바닥)
  python3 scripts/imu_lab.py drive --label carpet --secs 5           카펫 가장자리 넘기
  python3 scripts/imu_lab.py drive --label bump --secs 4             상자·쿠션에 살살 박기 (벽 말고!)
  python3 scripts/imu_lab.py drive --label spin --x 0 --z 0.3 --secs 5   제자리 회전
  python3 scripts/imu_lab.py table                                   지금까지 시험 전부 한 표로 비교

기록: ~/scr_logs/imu_lab/<시각>_<label>.csv (가속 g, 회전 °/s, 명령, 바퀴 속도)
숫자 읽는 법:
  흔들림(std)   = 평소에 얼마나 떨리나.  최대 튐 = 기준에서 가장 멀리 간 값.  최대 변화 = 한 칸(50 ms) 사이 가장 큰 변화(충격)
  축 이름은 보드 기준 (어느 축이 "앞" 인지는 floor/bump 결과로 확인)
안전: Nav2·충돌 감시 없이 직접 명령 → 짧게(기본 4초), 손 가까이. 스크립트가 죽어도 브리지가 0.5 초 뒤 멈춤.
"""
import argparse
import csv
import math
import statistics as st
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

OUT = Path.home() / "scr_logs" / "imu_lab"
G = 9.81
AX = ["ax", "ay", "az", "gx", "gy", "gz"]   # 가속 (g), 회전 (°/s)
BARS = " ▁▂▃▄▅▆▇█"


def record(mode, label, secs, x, z):
    rclpy.init()
    n = rclpy.create_node("imu_lab")
    rows, cmd, v = [], [0.0, 0.0], [0.0]

    def on_imu(m):
        a, w = m.linear_acceleration, m.angular_velocity
        rows.append([time.time(), a.x / G, a.y / G, a.z / G, math.degrees(w.x), math.degrees(w.y), math.degrees(w.z),
                     cmd[0], cmd[1], v[0]])
    n.create_subscription(Imu, "imu", on_imu, 50)
    n.create_subscription(Odometry, "odom", lambda m: v.__setitem__(0, m.twist.twist.linear.x), 10)
    pub = n.create_publisher(TwistStamped, "cmd_vel", 10)

    def send(cx, cz):
        cmd[0], cmd[1] = cx, cz
        t = TwistStamped()
        t.header.stamp = n.get_clock().now().to_msg()
        t.header.frame_id = "base_link"
        t.twist.linear.x, t.twist.angular.z = cx, cz
        pub.publish(t)

    # 연결 기다림 (11-07: Pi 에선 새 노드가 브리지와 연결되기까지 1~3 s — 그 전 명령은 아무도 못 받음)
    t_wait = time.time() + 6.0
    while time.time() < t_wait and (pub.get_subscription_count() == 0 or len(rows) < 5):
        rclpy.spin_once(n, timeout_sec=0.1)
    rows.clear()
    print(f"[{label}] 1초 기준 → {'움직임' if mode == 'drive' else '가만히'} {secs:.0f}초 → 1초 정리", flush=True)
    end_pre, end_act = time.time() + 1.0, time.time() + 1.0 + secs
    end_all = end_act + 1.0
    try:
        while time.time() < end_all:
            now = time.time()
            if mode == "drive":
                send(x, z) if end_pre <= now < end_act else send(0.0, 0.0)
            rclpy.spin_once(n, timeout_sec=0.05)
    except KeyboardInterrupt:
        pass
    finally:
        if mode == "drive":
            for _ in range(5):
                send(0.0, 0.0)
                time.sleep(0.02)
        n.destroy_node()
        rclpy.try_shutdown()

    if len(rows) < 10:
        print("IMU 기록이 거의 없음 — 브리지 v6 (/imu) 가 켜져 있나? `ros2 topic hz /imu`")
        return
    OUT.mkdir(parents=True, exist_ok=True)
    path = OUT / f"{time.strftime('%m%d_%H%M%S')}_{label}.csv"
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t"] + AX + ["cmd_x", "cmd_z", "wheel_v"])
        w.writerows(rows)
    t0 = rows[0][0]
    base = [r for r in rows if r[0] - t0 < 1.0]                   # 첫 1초 = 그 자리 기준
    act = [r for r in rows if 1.0 <= r[0] - t0 < 1.0 + secs]
    print(f"\n{path.name}  ({len(rows)} 줄, {len(rows) / (rows[-1][0] - t0):.0f} Hz)")
    print(summary(base, act))
    print(spark(rows, base))


def stats(rows, base):
    out = {}
    for i, k in enumerate(AX, start=1):
        b = st.mean(r[i] for r in base) if base else 0.0
        vals = [r[i] for r in rows]
        steps = [abs(vals[j] - vals[j - 1]) for j in range(1, len(vals))] or [0.0]
        out[k] = (st.pstdev(vals), max(abs(x - b) for x in vals), max(steps))
    return out


def summary(base, act):
    s = stats(act or base, base)
    lines = ["        흔들림(std)  최대 튐   최대 변화(50ms)"]
    for k in AX:
        u = "g " if k[0] == "a" else "°/s"
        lines.append(f"  {k} {u}  {s[k][0]:8.3f}  {s[k][1]:8.3f}  {s[k][2]:8.3f}")
    return "\n".join(lines)


def spark(rows, base):
    """수평 가속(ax, ay)이 기준에서 얼마나 벗어났나, 시간 순 막대 (한 칸 = 50 ms)"""
    bx = st.mean(r[1] for r in base) if base else 0.0
    by = st.mean(r[2] for r in base) if base else 0.0
    d = [math.hypot(r[1] - bx, r[2] - by) for r in rows]
    top = max(max(d), 0.05)
    bar = "".join(BARS[min(8, int(x / top * 8))] for x in d)
    return f"수평 가속 (최대 {top:.2f} g):\n  {bar}"


def table():
    files = sorted(OUT.glob("*.csv"))
    if not files:
        print("기록 없음:", OUT)
        return
    print(f"{'시험':28s} {'ax std':>7s} {'ax 튐':>6s} {'ay 튐':>6s} {'az 튐':>6s} {'gx 튐':>6s} {'gy 튐':>6s} {'gz 튐':>6s} {'a 변화':>7s}")
    for p in files:
        with open(p) as f:
            r = list(csv.reader(f))[1:]
        rows = [[float(x) for x in row] for row in r]
        if not rows:
            continue
        t0 = rows[0][0]
        base = [x for x in rows if x[0] - t0 < 1.0]
        s = stats(rows, base)
        jump = max(s["ax"][2], s["ay"][2], s["az"][2])
        print(f"{p.stem:28s} {s['ax'][0]:7.3f} {s['ax'][1]:6.2f} {s['ay'][1]:6.2f} {s['az'][1]:6.2f} "
              f"{s['gx'][1]:6.1f} {s['gy'][1]:6.1f} {s['gz'][1]:6.1f} {jump:7.2f}")
    print("(가속 g, 회전 °/s. 튐 = 첫 1초 기준에서 가장 멀리. a 변화 = 50 ms 사이 가장 큰 가속 변화 = 충격)")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("mode", choices=["still", "drive", "table"])
    ap.add_argument("--label", default="test")
    ap.add_argument("--secs", type=float, default=4.0)
    ap.add_argument("--x", type=float, default=0.08, help="m/s (drive)")
    ap.add_argument("--z", type=float, default=0.0, help="rad/s (drive)")
    a = ap.parse_args()
    if a.mode == "table":
        table()
    else:
        record(a.mode, a.label, a.secs, a.x, a.z)


if __name__ == "__main__":
    main()
