#!/usr/bin/env python3
"""[Pi] Whisker 실험실 — 로봇이 짧게 앞뒤로 계속 왕복하고, 사람은 아무 때나 손·발·상자를 넣어 본다.
터미널에 지금 무슨 일이 일어나는지 실시간으로 찍힌다. Ctrl+C 한 번 = 왕복도, Whisker 가 하던 일도 전부 멈춤.

  (먼저: robot 세션 + whisker 세션이 켜져 있어야 함)
  python3 scripts/whisker_lab.py                     # 앞 2 s (0.08 m/s ≈ 16 cm) ↔ 뒤 2 s (0.05 m/s ≈ 10 cm), 사이 1 s 쉼
  python3 scripts/whisker_lab.py --fwd 1.5 --back 1.5

찍히는 것
  [  12.3s] 명령 앞으로          ← 이 도구가 보내는 왕복 명령
  [  13.1s] ⚡ bump front (hard) 0.07 g     ← Whisker 가 느낀 것 (/sense)
  [  13.1s] ◆ stop  멈춤 (0.5 s)           ← Whisker 상태·계획 (/whisker/state)
  [  13.6s] ◆ back  빼기 — 뒤로 10 cm (hard)
  Whisker 가 처리하는 동안(stop/back/trapped/help)은 브리지가 이 도구의 명령을 무시한다 → 화면에 "(Whisker 처리 중)" 표시.
"""
import argparse
import json
import time

import rclpy
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Empty, String


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--fwd", type=float, default=2.0, help="앞으로 몇 초 (0.08 m/s)")
    ap.add_argument("--back", type=float, default=2.0, help="뒤로 몇 초 (0.05 m/s)")
    ap.add_argument("--pause", type=float, default=1.0, help="사이 쉼 (초)")
    a = ap.parse_args()

    # Ctrl+C 를 rclpy 가 먼저 잡으면 통신이 닫혀서 '멈춤' 명령을 못 보냄 → 우리가 직접 잡는다
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    n = rclpy.create_node("whisker_lab")
    cmd = n.create_publisher(TwistStamped, "cmd_vel", 10)
    resume = n.create_publisher(Empty, "whisker/resume", 1)
    t0 = time.time()
    wstate = {"state": "drive"}

    def say(text):
        print(f"[{time.time() - t0:6.1f}s] {text}", flush=True)

    def on_sense(m):
        d = json.loads(m.data)
        extra = d.get("g", d.get("dps", d.get("got", "")))
        say(f"⚡ {d.get('ev')} {d.get('dir', '')} ({d.get('feel', '')}) {extra}"
            + ("  → 지도 표시" if d.get("marked") else ""))

    def on_state(m):
        d = json.loads(m.data)
        wstate["state"] = d["state"]
        say(f"◆ {d['state']:7s} {d['msg']}")

    n.create_subscription(String, "sense", on_sense, 10)
    n.create_subscription(String, "whisker/state", on_state, 10)

    def send(x):
        t = TwistStamped()
        t.header.stamp = n.get_clock().now().to_msg()
        t.header.frame_id = "base_link"
        t.twist.linear.x = x
        cmd.publish(t)

    plan = [("앞으로", 0.08, a.fwd), ("쉼", 0.0, a.pause), ("뒤로", -0.05, a.back), ("쉼", 0.0, a.pause)]
    say("시작 — Ctrl+C 로 전부 멈춤. 아무 때나 손·발·상자를 넣어 보세요.")
    try:
        while True:
            for name, x, dur in plan:
                busy = wstate["state"] != "drive"
                say(f"명령 {name}" + ("   (Whisker 처리 중 — 이 명령은 무시됨)" if busy else ""))
                end = time.time() + dur
                while time.time() < end:
                    send(x)
                    rclpy.spin_once(n, timeout_sec=0.05)
    except KeyboardInterrupt:
        pass
    finally:
        for _ in range(10):                       # 왕복 멈춤
            send(0.0)
            time.sleep(0.02)
        resume.publish(Empty())                   # Whisker 가 하던 빼기·대기도 취소 → 정지
        time.sleep(0.2)
        print("\n멈춤: 왕복 0 + Whisker 취소 (resume)", flush=True)
        n.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
