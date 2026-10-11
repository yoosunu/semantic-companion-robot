#!/bin/bash
# H5 검증: burger LiDAR가 자기 하우징(lds.stl visual)에 가려지는지 확인한다.
# 원본 burger model.sdf를 복사해 두 변형을 만들고, 실행 중인 월드에 추가 spawn 후 /scan 값을 비교.
#   A: lidar_sensor_visual 제거          -> /scan_novis
#   B: 센서만 +0.06m 위로 (visual 유지)   -> /scan_up
# turtlebot3_ws 원본은 수정하지 않는다.
# 사용: robot-start 로 월드를 띄운 상태에서 실행

source /opt/ros/jazzy/setup.bash
source ~/turtlebot3_ws/install/setup.bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOG_DIR="$SCRIPT_DIR/../../logs"
TMP_DIR="$LOG_DIR/selfocclusion_models"
mkdir -p "$TMP_DIR"
OUT="$LOG_DIR/lidar_selfocclusion_test_$(date +%Y%m%d_%H%M%S).txt"
SRC=~/turtlebot3_ws/src/turtlebot3_simulations/turtlebot3_gazebo/models/turtlebot3_burger/model.sdf

python3 - "$SRC" "$TMP_DIR" <<'PY'
import sys, copy, xml.etree.ElementTree as ET
src, out = sys.argv[1], sys.argv[2]
base = ET.parse(src)

def variant(name, topic, remove_visual, raise_z):
    t = copy.deepcopy(base)
    model = t.getroot().find('model')
    model.set('name', name)
    for p in model.findall('plugin'):          # diff drive / joint state 등 제거 (원래 로봇과 토픽 충돌 방지)
        model.remove(p)
    link = [l for l in model.findall('link') if l.get('name') == 'base_scan'][0]
    if remove_visual:
        for v in link.findall('visual'):
            link.remove(v)
    s = link.find('sensor')
    s.find('topic').text = topic
    if raise_z:
        pose = s.find('pose')
        v = [float(x) for x in pose.text.split()]
        v[2] += raise_z
        pose.text = ' '.join(str(x) for x in v)
    t.write(f"{out}/{name}.sdf")

variant('burger_novis', 'scan_novis', True, 0.0)
variant('burger_up',    'scan_up',    False, 0.06)
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
  ros2 run ros_gz_sim create -world default -file "$TMP_DIR/burger_novis.sdf" -name burger_novis -x -2.0 -y  0.0 -z 0.01 2>&1 | tail -n 2
  ros2 run ros_gz_sim create -world default -file "$TMP_DIR/burger_up.sdf"    -name burger_up    -x -2.0 -y -1.0 -z 0.01 2>&1 | tail -n 2
  sleep 6

  echo; echo "### gz scan topics"; gz topic -l | grep -i scan

  for t in /scan /scan_novis /scan_up; do
    echo; echo "### $t"
    msg="$(timeout 10 gz topic -e -t $t -n 1 2>&1)"
    if [ -z "$msg" ]; then echo "no message"; continue; fi
    echo "$msg" | stats
    echo "every 30deg: $(echo "$msg" | awk '/^ranges:/{i++; if ((i-1)%30==0) printf "%.2f ", $2}')"
  done
  echo "=== done ==="
} > "$OUT" 2>&1

echo "saved: $OUT"
