#!/bin/bash
# burger LiDAR를 '정상 동작이 확인된' 공식 예제 월드(gpu_lidar_sensor.sdf, ogre, 서버 단독)에 넣어본다.
#   control : 예제 월드 자체 LiDAR (/lidar)          -> 정상이어야 함
#   stock   : burger LiDAR 그대로 (수직 1줄)          -> /scan_stock
#   vert16  : burger LiDAR + 수직 16줄 (±15°)         -> /scan_vert16
# 판정:
#   stock 정상  -> 문제는 TurtleBot3 월드/launch 쪽
#   stock 실패, vert16 정상 -> 수직 1줄(높이 1px 렌더링)이 원인
#   둘 다 실패 -> burger 센서 정의의 다른 요소
# 사용: robot-stop 으로 기존 시뮬레이션을 끈 뒤 실행

source /opt/ros/jazzy/setup.bash
source ~/turtlebot3_ws/install/setup.bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOG_DIR="$SCRIPT_DIR/../../logs"
TMP_DIR="$LOG_DIR/lidar_world_models"
mkdir -p "$TMP_DIR"
OUT="$LOG_DIR/lidar_world_test_$(date +%Y%m%d_%H%M%S).txt"
MODELS=~/turtlebot3_ws/src/turtlebot3_simulations/turtlebot3_gazebo/models
SRC=$MODELS/turtlebot3_burger/model.sdf
export GZ_SIM_RESOURCE_PATH="$MODELS:$GZ_SIM_RESOURCE_PATH"

python3 - "$SRC" "$TMP_DIR" <<'PY'
import sys, copy, xml.etree.ElementTree as ET
src, out = sys.argv[1], sys.argv[2]
base = ET.parse(src)

def make(name, edit=None):
    t = copy.deepcopy(base)
    model = t.getroot().find('model')
    model.set('name', f'burger_{name}')
    for p in model.findall('plugin'):
        model.remove(p)
    link = [l for l in model.findall('link') if l.get('name') == 'base_scan'][0]
    s = link.find('sensor')
    s.find('topic').text = f'scan_{name}'
    if edit: edit(s.find('lidar'))
    t.write(f"{out}/{name}.sdf")

def vert16(lidar):
    scan = lidar.find('scan')
    v = ET.SubElement(scan, 'vertical')
    for k, val in (('samples', '16'), ('resolution', '1'), ('min_angle', '-0.2618'), ('max_angle', '0.2618')):
        ET.SubElement(v, k).text = val

make('stock')
make('vert16', vert16)
print("variants written")
PY

stats() {
  awk '/^ranges:/ {
         n++; v=$2
         if (v=="inf" || v=="-inf") inf++
         else { f++; if (min=="" || v+0<min) min=v+0; if (v+0>max) max=v+0; seen[v]=1 }
       }
       END { d=0; for (k in seen) d++
             printf "beams=%d finite=%d inf=%d min=%s max=%s distinct=%d\n", n, f, inf, min, max, d }'
}

{
  echo "=== $(date) ==="
  pkill -f "gz sim" 2>/dev/null; sleep 2
  gz sim -s -r -v 3 gpu_lidar_sensor.sdf --render-engine-server ogre > "$LOG_DIR/.gz_server.log" 2>&1 &
  PID=$!
  sleep 8
  kill -0 $PID 2>/dev/null || { echo "server died"; tail -n 20 "$LOG_DIR/.gz_server.log"; exit 1; }

  echo "### spawn"
  ros2 run ros_gz_sim create -world gpu_lidar_sensor -file "$TMP_DIR/stock.sdf"  -name burger_stock  -x 0 -y  1.5 -z 0.01 2>&1 | tail -n 1
  ros2 run ros_gz_sim create -world gpu_lidar_sensor -file "$TMP_DIR/vert16.sdf" -name burger_vert16 -x 0 -y -1.5 -z 0.01 2>&1 | tail -n 1
  sleep 6

  for t in /lidar /scan_stock /scan_vert16; do
    echo; echo "### $t"
    msg="$(timeout 10 gz topic -e -t $t -n 1 2>&1)"
    if [ -z "$msg" ]; then echo "no message"; continue; fi
    echo "$msg" | stats
    echo "first 10: $(echo "$msg" | awk '/^ranges:/{i++; if (i<=10) printf "%.3f ", $2}')"
  done

  echo; echo "--- server log (warnings/errors) ---"
  grep -iE "error|warn" "$LOG_DIR/.gz_server.log" | tail -n 10

  kill $PID 2>/dev/null; sleep 2; kill -9 $PID 2>/dev/null
  rm -f "$LOG_DIR/.gz_server.log"
  echo "=== done ==="
} > "$OUT" 2>&1

echo "saved: $OUT"
