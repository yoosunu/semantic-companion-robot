#!/bin/bash
# 다중 행 gpu_lidar의 (1) gz ranges 배열 순서, (2) ros_gz_bridge가 ROS로 넘기는 행, (3) 실제 거리 정확도를
# TurtleBot3 월드(기둥 위치를 아는 환경)에서 확정한다.
#   v3wide : 수직 3줄 [-0.6, -0.3, 0] rad. 맨 아래 줄은 바닥(약 0.26m)에 닿음 -> 배열 순서 판별용
#   v3     : 수직 3줄 ±0.5°, 가운데 0°   -> 실제 사용 후보, (-2.0, 0.5)에 놓고 기둥 거리로 정확도 확인
# 사용: robot-start 로 월드를 띄운 상태에서 실행

source /opt/ros/jazzy/setup.bash
source ~/turtlebot3_ws/install/setup.bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOG_DIR="$SCRIPT_DIR/../../logs"
TMP_DIR="$LOG_DIR/lidar_rows_probe2"
mkdir -p "$TMP_DIR"
OUT="$LOG_DIR/lidar_rows_probe2_$(date +%Y%m%d_%H%M%S).txt"
SRC=~/turtlebot3_ws/src/turtlebot3_simulations/turtlebot3_gazebo/models/turtlebot3_burger/model.sdf

python3 - "$SRC" "$TMP_DIR" <<'PY'
import sys, copy, xml.etree.ElementTree as ET
src, out = sys.argv[1], sys.argv[2]
base = ET.parse(src)
def make(name, lo, hi):
    t = copy.deepcopy(base)
    model = t.getroot().find('model'); model.set('name', f'burger_{name}')
    for p in model.findall('plugin'): model.remove(p)
    link = [l for l in model.findall('link') if l.get('name') == 'base_scan'][0]
    s = link.find('sensor'); s.find('topic').text = f'scan_{name}'
    lidar = s.find('lidar'); lidar.remove(lidar.find('noise'))   # 판별을 깨끗하게 하려고 노이즈 제거
    v = ET.SubElement(lidar.find('scan'), 'vertical')
    for k, val in (('samples', '3'), ('resolution', '1'), ('min_angle', str(lo)), ('max_angle', str(hi))):
        ET.SubElement(v, k).text = val
    t.write(f"{out}/{name}.sdf")
make('v3wide', -0.6, 0.0)
make('v3', -0.0087, 0.0087)
PY

{
  echo "=== $(date) ==="
  ros2 run ros_gz_sim create -world default -file "$TMP_DIR/v3wide.sdf" -name burger_v3wide -x -2.0 -y 1.0 -z 0.01 2>&1 | tail -n 1
  ros2 run ros_gz_sim create -world default -file "$TMP_DIR/v3.sdf"     -name burger_v3     -x -2.0 -y 0.5 -z 0.01 2>&1 | tail -n 1
  ros2 run ros_gz_bridge parameter_bridge \
      /scan_v3wide@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan \
      /scan_v3@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan > /dev/null 2>&1 &
  BPID=$!
  sleep 6

  for n in v3wide v3; do
    timeout 10 gz topic -e -t /scan_$n -n 1 | awk '/^ranges:/{print $2}' > "$TMP_DIR/$n.gz"
  done

  timeout 15 python3 - "$TMP_DIR" <<'PY'
import sys, math, rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
d = sys.argv[1]
rclpy.init(); node = Node('rows_probe2'); got = {}
for n in ('v3wide', 'v3'):
    node.create_subscription(LaserScan, f'/scan_{n}', lambda m, n=n: got.setdefault(n, m), qos_profile_sensor_data)
while rclpy.ok() and len(got) < 2: rclpy.spin_once(node, timeout_sec=0.5)

NH, NV = 360, 3
def f(x): return x if math.isfinite(x) else float('nan')
def row(vals, j, layout):
    return [vals[j*NH+i] for i in range(NH)] if layout == 'row' else [vals[i*NV+j] for i in range(NH)]
def stat(xs):
    xs = [x for x in xs if math.isfinite(x)]
    if not xs: return "no finite"
    m = sum(xs)/len(xs); sd = (sum((x-m)**2 for x in xs)/len(xs))**0.5
    return f"n={len(xs)} mean={m:.3f} sd={sd:.3f} min={min(xs):.3f} max={max(xs):.3f}"
def mad(a, b):
    ds = [abs(x-y) for x, y in zip(a, b) if math.isfinite(x) and math.isfinite(y)]
    return (sum(ds)/len(ds), len(ds)) if ds else (float('nan'), 0)

print("### (1) gz layout — v3wide, rows [-0.6, -0.3, 0.0] rad, bottom row should hit floor ~0.26m everywhere")
w = [float(x) for x in open(f"{d}/v3wide.gz").read().split()]
print(f"  gz total={len(w)}")
for layout in ('row', 'col'):
    for j in range(NV):
        print(f"  layout={layout:3s} row{j}: {stat(row(w, j, layout))}")

print("\n### (2) which gz row does ros_gz_bridge pass to ROS?")
for n in ('v3wide', 'v3'):
    g = [float(x) for x in open(f"{d}/{n}.gz").read().split()]
    r = list(got[n].ranges)
    print(f"  {n}: ROS len={len(r)}")
    for layout in ('row', 'col'):
        for j in range(NV):
            m, k = mad(r, row(g, j, layout))
            print(f"    vs gz layout={layout:3s} row{j}: mean|diff|={m:.4f} (n={k})")
    print(f"    vs gz first {NH} raw elements: mean|diff|={mad(r, g[:NH])[0]:.4f}")

print("\n### (3) accuracy — v3 at (-2.0, 0.5), yaw 0. Expected nearest pillars ~0.9m near -29deg(331) and +34deg")
r = list(got['v3'].ranges)
print(f"  ROS v3: {stat(r)}")
mins = []
for i in range(NH):
    a, b, c = r[(i-1) % NH], r[i], r[(i+1) % NH]
    if math.isfinite(b) and b <= a and b <= c and b < 2.0:
        mins.append((b, i))
mins.sort()
print("  local minima (<2m) [range, deg]:", [(round(v, 3), i) for v, i in mins[:8]])
print("  every 30deg:", [round(r[i], 2) if math.isfinite(r[i]) else 'inf' for i in range(0, NH, 30)])
node.destroy_node(); rclpy.shutdown()
PY
  kill $BPID 2>/dev/null
  echo "=== done ==="
} > "$OUT" 2>&1
echo "saved: $OUT"
