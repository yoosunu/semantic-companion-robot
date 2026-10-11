#!/bin/bash
# SLAM 실행 중 상태를 한 번에 점검해서 logs/에 저장한다.
# 사용: robot-start, robot-slam 실행 후 다른 터미널에서 ./scripts/slam_check.sh

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/env.sh"
LOG_DIR="$SCRIPT_DIR/../logs"
mkdir -p "$LOG_DIR"
OUT="$LOG_DIR/slam_check_$(date +%Y%m%d_%H%M%S).txt"

# 노드 탐색은 daemon 캐시가 불안정할 수 있어 --no-daemon으로 한 번만 조회
NODES="$(ros2 node list --no-daemon 2>/dev/null)"
EXPECTED="/cartographer_node /cartographer_occupancy_grid_node /robot_state_publisher /ros_gz_bridge /rviz2"

{
  echo "=== $(date) ==="
  echo; echo "### nodes (--no-daemon)"; echo "$NODES"

  echo; echo "### expected nodes"
  for n in $EXPECTED; do
    if echo "$NODES" | grep -qx "$n"; then echo "OK      $n"; else echo "MISSING $n"; fi
  done

  echo; echo "### use_sim_time"
  for n in $EXPECTED; do
    printf "%-38s " "$n"; timeout 5 ros2 param get "$n" use_sim_time 2>&1 | tail -n1
  done

  echo; echo "### /map publishers / subscribers"; ros2 topic info /map 2>&1

  echo; echo "### /scan hz";  timeout 6 ros2 topic hz /scan  2>&1 | tail -n 2
  echo; echo "### /odom hz";  timeout 6 ros2 topic hz /odom  2>&1 | tail -n 2

  echo; echo "### /scan content (one message)"
  timeout 10 python3 - <<'PY'
import math, rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
rclpy.init()
n = Node('scan_probe')
got = []
n.create_subscription(LaserScan, '/scan', lambda m: got.append(m), qos_profile_sensor_data)
while rclpy.ok() and not got:
    rclpy.spin_once(n, timeout_sec=0.5)
m = got[0]
r = list(m.ranges)
fin = [x for x in r if math.isfinite(x)]
valid = [x for x in fin if m.range_min <= x <= m.range_max]
print(f"frame_id={m.header.frame_id} stamp={m.header.stamp.sec}.{m.header.stamp.nanosec:09d}")
print(f"angle_min={m.angle_min:.3f} angle_max={m.angle_max:.3f} increment={m.angle_increment:.4f}")
print(f"range_min={m.range_min} range_max={m.range_max}")
print(f"beams={len(r)} finite={len(fin)} valid_in_range={len(valid)} inf={sum(1 for x in r if math.isinf(x))} nan={sum(1 for x in r if math.isnan(x))}")
if valid:
    print(f"valid min={min(valid):.3f} max={max(valid):.3f} mean={sum(valid)/len(valid):.3f}")
step = max(1, len(r)//12)
print("sample every 30deg:", [round(r[i],2) for i in range(0, len(r), step)])
n.destroy_node(); rclpy.shutdown()
PY

  echo; echo "### /map info"; timeout 6 ros2 topic echo --once /map --field info 2>&1
  echo; echo "### tf map->odom";            timeout 4 ros2 run tf2_ros tf2_echo map odom 2>&1 | grep -E "At time|Translation|degree" | head -n 3
  echo; echo "### tf odom->base_footprint"; timeout 4 ros2 run tf2_ros tf2_echo odom base_footprint 2>&1 | grep -E "At time|Translation|degree" | head -n 3
  echo; echo "### tf base_link->base_scan"; timeout 4 ros2 run tf2_ros tf2_echo base_link base_scan 2>&1 | grep -E "At time|Translation|degree" | head -n 3
} > "$OUT" 2>&1

echo "saved: $OUT"
