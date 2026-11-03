#!/usr/bin/env python3
"""[Pi] 안전 회귀 시험: 바퀴가 도는 중 브리지가 죽거나(crash) 얼면(hang) 펌웨어가 스스로 멈추는가.

  바퀴를 띄우고, ROS 노드를 다 끈 상태에서:
  source ~/scr_ws/install/setup.bash && python3 scripts/failsafe_test.py crash   # kill -9
                                        python3 scripts/failsafe_test.py hang    # kill -STOP
                                        python3 scripts/failsafe_test.py crash pwm   # 브리지 v5 PWM 모드로
  통과: 0.5~0.8 s 안에 L, R 이 계속 0.  (11-03 v4: 둘 다 0.65 s. 배경: docs/debugging/004)
  브리지 v4 의 heartbeat(T:136 500 ms) + T:1 주행이 바뀌면 다시 돌릴 것.
"""
import subprocess, time, signal, os, sys, json, serial

mode = sys.argv[1] if len(sys.argv) > 1 else ""
drive = sys.argv[2] if len(sys.argv) > 2 else "pid"
if mode not in ("crash", "hang"):
    sys.exit(__doc__)
BR = os.path.expanduser("~/scr_ws/install/scr_ugv02/lib/scr_ugv02/ugv02_bridge")
br = subprocess.Popen([BR, "--ros-args", "-p", "port:=/dev/ttyAMA0", "-p", f"drive_mode:={drive}"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
time.sleep(5)                                        # 자이로 영점 3초 + 여유
pub = subprocess.Popen(["ros2", "topic", "pub", "-r", "10", "/cmd_vel", "geometry_msgs/msg/TwistStamped",
                        "{twist: {linear: {x: 0.1}}}"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
time.sleep(4)
os.kill(br.pid, signal.SIGKILL if mode == "crash" else signal.SIGSTOP)
t0 = time.time()
print(f"[{mode} / {drive}] 바퀴 도는 중에 브리지 {'강제 종료' if mode == 'crash' else '얼림'} → 3초간 바퀴 속도")
s = serial.Serial('/dev/ttyAMA0', 115200, timeout=0.1); s.reset_input_buffer(); last = -1
stopped_at = None
try:
    while time.time() - t0 < 3:
        try: d = json.loads(s.readline())
        except Exception: continue
        if d.get('T') != 1001: continue
        moving = abs(d.get('L', 0)) > 1e-3 or abs(d.get('R', 0)) > 1e-3
        if moving: stopped_at = None
        elif stopped_at is None: stopped_at = time.time() - t0
        k = int((time.time() - t0) * 8)
        if k != last: last = k; print(f"{time.time()-t0:4.2f}s  L={d.get('L')}  R={d.get('R')}")
finally:
    pub.terminate()
    if mode == "hang": os.kill(br.pid, signal.SIGKILL)
    for _ in range(3): s.write(b'{"T":1,"L":0,"R":0}\n'); time.sleep(0.05)
    ok = stopped_at is not None and stopped_at < 1.0
    print(f"정지 시각: {stopped_at if stopped_at is None else round(stopped_at, 2)} s → {'통과' if ok else '실패'}")
