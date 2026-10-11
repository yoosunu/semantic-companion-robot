#!/bin/bash
# burger gpu_lidar 파라미터를 한 번에 하나씩 바꾼 변형을 spawn해서 어떤 요인이 0.12 고정을 만드는지 찾는다.
#   fov180  : 수평 시야각 360° -> 180° (samples 180)
#   fov90   : 수평 시야각 360° -> 90°  (samples 90)
#   nonoise : noise 제거 (360° 유지)
#   min008  : range min 0.12 -> 0.08 (360° 유지)
# turtlebot3_ws 원본은 수정하지 않는다.
# 사용: robot-stop -> robot-start 로 깨끗한 월드를 띄운 뒤 실행

source /opt/ros/jazzy/setup.bash
source ~/turtlebot3_ws/install/setup.bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOG_DIR="$SCRIPT_DIR/../../logs"
TMP_DIR="$LOG_DIR/lidar_param_models"
mkdir -p "$TMP_DIR"
OUT="$LOG_DIR/lidar_param_test_$(date +%Y%m%d_%H%M%S).txt"
SRC=~/turtlebot3_ws/src/turtlebot3_simulations/turtlebot3_gazebo/models/turtlebot3_burger/model.sdf

python3 - "$SRC" "$TMP_DIR" <<'PY'
import sys, copy, xml.etree.ElementTree as ET
src, out = sys.argv[1], sys.argv[2]
base = ET.parse(src)

def make(name, edit):
    t = copy.deepcopy(base)
    model = t.getroot().find('model')
    model.set('name', f'burger_{name}')
    for p in model.findall('plugin'):
        model.remove(p)
    link = [l for l in model.findall('link') if l.get('name') == 'base_scan'][0]
    s = link.find('sensor')
    s.find('topic').text = f'scan_{name}'
    edit(s.find('lidar'))
    t.write(f"{out}/{name}.sdf")

def fov(lo, hi, n):
    def e(lidar):
        h = lidar.find('scan/horizontal')
        h.find('samples').text = str(n)
        h.find('min_angle').text = str(lo)
        h.find('max_angle').text = str(hi)
    return e

def nonoise(lidar):
    lidar.remove(lidar.find('noise'))

def min008(lidar):
    lidar.find('range/min').text = '0.08'

make('fov180',  fov(-1.5708, 1.5708, 180))
make('fov90',   fov(-0.7854, 0.7854, 90))
make('nonoise', nonoise)
make('min008',  min008)
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
  echo "### spawn"
  Y=(0.0 -1.0 0.5 -1.5)
  i=0
  for v in fov180 fov90 nonoise min008; do
    ros2 run ros_gz_sim create -world default -file "$TMP_DIR/$v.sdf" -name "burger_$v" -x -2.0 -y "${Y[$i]}" -z 0.01 2>&1 | tail -n 1
    i=$((i+1))
  done
  sleep 6

  for t in /scan /scan_fov180 /scan_fov90 /scan_nonoise /scan_min008; do
    echo; echo "### $t"
    msg="$(timeout 10 gz topic -e -t $t -n 1 2>&1)"
    if [ -z "$msg" ]; then echo "no message"; continue; fi
    echo "$msg" | stats
    echo "first 10: $(echo "$msg" | awk '/^ranges:/{i++; if (i<=10) printf "%.3f ", $2}')"
  done
  echo "=== done ==="
} > "$OUT" 2>&1

echo "saved: $OUT"
