#!/usr/bin/env python3
"""cpu_probe — 프로세스별 CPU 를 몇 초 평균으로 (11-13). python3 cpu_probe.py [초=20] [이름]

  /proc/<pid>/stat 의 사용 시간 변화로 계산 → "코어 1 개 = 100%" (4 코어 Pi 5 는 전체 400%)
  우리 것만 골라서 묶음: 아래 GROUPS 의 글자가 명령줄에 들어 있으면 그 묶음.
  같이: 전체 CPU (4 코어 평균 %), 온도, 스로틀 여부 (vcgencmd), Pi 전력 (power_probe 와 같은 PMIC 합)
"""
import os
import re
import subprocess
import sys
import time

GROUPS = [  # (보여줄 이름, 명령줄에 들어 있는 글자)
    ("보기 face_perception", "face_perception.py"),
    ("고개 look", "look_at_person.py"),
    ("YOLO yolo_lab", "yolo_lab.py"),
    ("바닥 floor_lab", "floor_lab.py"),
    ("얼굴실험 face_lab", "face_lab.py"),
    ("카메라 camera_node", "camera_node"),
    ("브리지 ugv02_bridge", "ugv02_bridge"),
    ("LiDAR sllidar", "sllidar"),
    ("SLAM slam_toolbox", "slam_toolbox"),
    ("Nav2 (컨테이너·서버)", "nav2"),
    ("Nav2 component_container", "component_container"),
    ("탐색 scr_explore", "scr_explore.py"),
    ("위스커 whisker", "whisker.py"),
    ("상태 status_display", "status_display.py"),
    ("ROS 기타 (robot_state_publisher 등)", "robot_state_publisher"),
]
HZ = os.sysconf("SC_CLK_TCK")


def snap():
    out = {}
    for pid in os.listdir("/proc"):
        if not pid.isdigit():
            continue
        try:
            cmd = open(f"/proc/{pid}/cmdline", "rb").read().replace(b"\0", b" ").decode(errors="ignore")
            f = open(f"/proc/{pid}/stat").read().rsplit(")", 1)[1].split()
            out[int(pid)] = (cmd, int(f[11]) + int(f[12]))           # utime + stime (틱)
        except (OSError, IndexError):
            pass
    return out


def total():
    n = list(map(int, open("/proc/stat").readline().split()[1:]))
    return sum(n), n[3] + n[4]


def vc(*args):
    try:
        return subprocess.run(["vcgencmd", *args], capture_output=True, text=True, timeout=2).stdout.strip()
    except (OSError, subprocess.SubprocessError):
        return ""


def pmic():
    out = vc("pmic_read_adc")
    a, v = {}, {}
    for k, kind, val in re.findall(r"(\S+)_([AV]) \w+\(\d+\)=([\d.]+)", out):
        (a if kind == "A" else v)[k] = float(val)
    return sum(a[k] * v[k] for k in a if k in v)


secs = float(sys.argv[1]) if len(sys.argv) > 1 else 20
name = sys.argv[2] if len(sys.argv) > 2 else ""
s0, (t0, i0), w0 = snap(), total(), time.time()
watts = []
while time.time() - w0 < secs:
    p = pmic()
    if p:
        watts.append(p)
    time.sleep(1.0)
s1, (t1, i1), dt = snap(), total(), time.time() - w0
use = {}
for pid, (cmd, tk) in s1.items():
    if pid not in s0 or s0[pid][0] != cmd:
        continue
    d = (tk - s0[pid][1]) / HZ / dt * 100.0
    if d <= 0:
        continue
    for g, key in GROUPS:
        if key in cmd:
            use[g] = use.get(g, 0.0) + d
            break
    else:
        use.setdefault("(그 밖)", 0.0)
        use["(그 밖)"] += d
all_pct = 100.0 * (1 - (i1 - i0) / max(1, t1 - t0))
temp = vc("measure_temp").replace("temp=", "")
thr = vc("get_throttled").replace("throttled=", "")
print(f"=== {name}  ({dt:.0f} s)  전체 {all_pct:.0f}% (4코어 평균) = 코어 {all_pct * 4 / 100:.2f} 개 | 온도 {temp} | 스로틀 {thr}"
      + (f" | Pi {sum(watts) / len(watts):.2f} W" if watts else ""))
for g, d in sorted(use.items(), key=lambda x: -x[1]):
    if d >= 0.5:
        print(f"  {g:<34} {d:6.1f}%  (코어 1개 기준)")
