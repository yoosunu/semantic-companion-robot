#!/bin/bash
# 수정 방향 결정을 위한 사전 확인.
#  1) 수직 2줄 / 3줄 gpu_lidar가 이 VM에서 동작하는지
#  2) 다중 행 ranges 배열이 행 우선(row-major)인지 열 우선(column-major)인지
#  3) ros_gz_bridge를 거친 ROS LaserScan의 ranges 길이와 angle 필드
#  4) TurtleBot3 launch/params/urdf 참고 파일을 logs/tb3_ref 로 복사
# 사용: robot-stop 후 실행

source /opt/ros/jazzy/setup.bash
source ~/turtlebot3_ws/install/setup.bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOG_DIR="$SCRIPT_DIR/../../logs"
TMP_DIR="$LOG_DIR/lidar_rows_models"
REF_DIR="$LOG_DIR/tb3_ref"
mkdir -p "$TMP_DIR" "$REF_DIR"
OUT="$LOG_DIR/lidar_rows_probe_$(date +%Y%m%d_%H%M%S).txt"
PKG=~/turtlebot3_ws/src/turtlebot3_simulations/turtlebot3_gazebo
MODELS=$PKG/models
export GZ_SIM_RESOURCE_PATH="$MODELS:$GZ_SIM_RESOURCE_PATH"

# 4) 참고 파일 복사
for d in launch params urdf; do [ -d "$PKG/$d" ] && cp -r "$PKG/$d" "$REF_DIR/"; done
cp "$PKG/package.xml" "$PKG/CMakeLists.txt" "$REF_DIR/" 2>/dev/null
ls -R "$PKG" | grep -v "\.stl$\|\.dae$\|\.png$" > "$REF_DIR/_tree.txt"

python3 - "$MODELS/turtlebot3_burger/model.sdf" "$TMP_DIR" <<'PY'
import sys, copy, xml.etree.ElementTree as ET
src, out = sys.argv[1], sys.argv[2]
base = ET.parse(src)
def make(name, rows, half):
    t = copy.deepcopy(base)
    model = t.getroot().find('model'); model.set('name', f'burger_{name}')
    for p in model.findall('plugin'): model.remove(p)
    link = [l for l in model.findall('link') if l.get('name') == 'base_scan'][0]
    s = link.find('sensor'); s.find('topic').text = f'scan_{name}'
    scan = s.find('lidar/scan')
    v = ET.SubElement(scan, 'vertical')
    for k, val in (('samples', str(rows)), ('resolution', '1'), ('min_angle', str(-half)), ('max_angle', str(half))):
        ET.SubElement(v, k).text = val
    t.write(f"{out}/{name}.sdf")
make('vert2', 2, 0.0087)   # ±0.5°
make('vert3', 3, 0.0087)   # ±0.5°, 가운데 행 = 0°
PY

analyze() {  # $1 = file with one range per line, $2 = rows
python3 - "$1" "$2" <<'PY'
import sys, math
vals = [float(x) for x in open(sys.argv[1]).read().split()]
nv = int(sys.argv[2]); n = len(vals); nh = n // nv
fin = [v for v in vals if math.isfinite(v)]
print(f"  total={n} rows={nv} per_row={nh} finite={len(fin)} min={min(fin) if fin else None:.3f} max={max(fin) if fin else None:.3f} distinct={len(set(fin))}")
def mad(pairs):
    d = [abs(a-b) for a,b in pairs if math.isfinite(a) and math.isfinite(b)]
    return sum(d)/len(d) if d else float('nan')
row_major = mad((vals[j*nh+i], vals[(j+1)*nh+i]) for j in range(nv-1) for i in range(nh))
col_major = mad((vals[i*nv+j], vals[i*nv+j+1]) for i in range(nh) for j in range(nv-1))
print(f"  mean|diff| assuming row-major={row_major:.4f}  column-major={col_major:.4f}  -> smaller one is the layout")
print(f"  first 6: {[round(v,3) for v in vals[:6]]}")
PY
}

{
  echo "=== $(date) ==="
  pkill -f "gz sim" 2>/dev/null; sleep 2
  gz sim -s -r -v 3 gpu_lidar_sensor.sdf --render-engine-server ogre > "$LOG_DIR/.gz_server.log" 2>&1 &
  PID=$!
  sleep 8
  kill -0 $PID 2>/dev/null || { echo "server died"; tail -n 20 "$LOG_DIR/.gz_server.log"; exit 1; }

  echo "### spawn"
  ros2 run ros_gz_sim create -world gpu_lidar_sensor -file "$TMP_DIR/vert2.sdf" -name burger_vert2 -x 0 -y  1.5 -z 0.01 2>&1 | tail -n 1
  ros2 run ros_gz_sim create -world gpu_lidar_sensor -file "$TMP_DIR/vert3.sdf" -name burger_vert3 -x 0 -y -1.5 -z 0.01 2>&1 | tail -n 1
  sleep 6

  for c in "vert2 2" "vert3 3"; do
    set -- $c
    echo; echo "### gz /scan_$1"
    timeout 10 gz topic -e -t /scan_$1 -n 1 > "$TMP_DIR/$1.gzmsg" 2>&1
    grep -E "^(count|vertical_count|angle_min|angle_max|vertical_angle_min|vertical_angle_max|range_min|range_max):" "$TMP_DIR/$1.gzmsg"
    awk '/^ranges:/{print $2}' "$TMP_DIR/$1.gzmsg" > "$TMP_DIR/$1.ranges"
    analyze "$TMP_DIR/$1.ranges" $2
  done

  echo; echo "### ROS side via ros_gz_bridge (/scan_vert3)"
  ros2 run ros_gz_bridge parameter_bridge /scan_vert3@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan > /dev/null 2>&1 &
  BPID=$!
  sleep 4
  timeout 10 python3 - <<'PY'
import math, rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
rclpy.init(); n = Node('rows_probe'); got = []
n.create_subscription(LaserScan, '/scan_vert3', lambda m: got.append(m), qos_profile_sensor_data)
while rclpy.ok() and not got: rclpy.spin_once(n, timeout_sec=0.5)
m = got[0]
print(f"  frame_id={m.header.frame_id} len(ranges)={len(m.ranges)} len(intensities)={len(m.intensities)}")
print(f"  angle_min={m.angle_min:.4f} angle_max={m.angle_max:.4f} angle_increment={m.angle_increment:.5f} -> implied beams={round((m.angle_max-m.angle_min)/m.angle_increment)+1}")
n.destroy_node(); rclpy.shutdown()
PY
  kill $BPID 2>/dev/null

  echo; echo "### reference files copied"; ls "$REF_DIR" "$REF_DIR"/* 2>/dev/null | head -n 40

  kill $PID 2>/dev/null; sleep 2; kill -9 $PID 2>/dev/null
  rm -f "$LOG_DIR/.gz_server.log"
  echo "=== done ==="
} > "$OUT" 2>&1

echo "saved: $OUT"
