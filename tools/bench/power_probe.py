#!/usr/bin/env python3
"""Pi 5 내부 전력(PMIC 레일 전류×전압 합)과 CPU 사용률을 몇 초 평균. python3 power_probe.py [초] [이름]
   PMIC 는 SoC·메모리·Wi-Fi 등 Pi 안쪽만. USB 로 먹는 LiDAR·카메라 센서 자체는 안 잡힘 (LiDAR 는 스펙 1.15 W)."""
import re
import subprocess
import sys
import time

secs = float(sys.argv[1]) if len(sys.argv) > 1 else 20
name = sys.argv[2] if len(sys.argv) > 2 else ""


def pmic():
    out = subprocess.run(["vcgencmd", "pmic_read_adc"], capture_output=True, text=True).stdout
    a, v = {}, {}
    for k, kind, val in re.findall(r"(\S+)_([AV]) \w+\(\d+\)=([\d.]+)", out):
        (a if kind == "A" else v)[k] = float(val)
    return sum(a[k] * v[k] for k in a if k in v)


def cpu():
    f = open("/proc/stat").readline().split()[1:]
    n = list(map(int, f))
    return sum(n), n[3] + n[4]          # 전체, idle+iowait


ps, (t0, i0) = [], cpu()
end = time.time() + secs
while time.time() < end:
    ps.append(pmic())
    time.sleep(0.5)
t1, i1 = cpu()
use = 100.0 * (1 - (i1 - i0) / max(1, t1 - t0))
if not ps or max(ps) == 0:
    print("PMIC 못 읽음 (vcgencmd 권한? → sudo 로 또는 video 그룹)"); sys.exit(1)
print(f"{name:<28} Pi {sum(ps)/len(ps):5.2f} W (최대 {max(ps):5.2f})   CPU {use:5.1f}% (4코어 평균)")
