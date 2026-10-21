#!/bin/bash
# Nav2 기준선 테스트: 위치 추정(AMCL)과 목표 주행을 숫자로 판정해 logs/에 저장한다.
# 사용: robot_nav.sh 실행 중에  ./scripts/nav_test.sh [goal_x goal_y [goal_yaw_deg [pillar_x pillar_y]]]
#   (map 좌표, 기본 3.6 0.0 0). pillar 를 주면 도착 후 로봇 중심 ~ 기둥 중심 거리도 출력 (미션 L2).
#   TB3 월드 기둥 9개의 map 좌표: x ∈ {0.9, 2.0, 3.1}, y ∈ {-0.6, 0.5, 1.6}  (월드 ±1.1 격자 + 오프셋 2.0, 0.5)
#
# 정답 위치: Gazebo의 실제 로봇 위치(gz model). map 원점 = 로봇 출발점(월드 -2.0, -0.5, yaw 0)이라
#            map 좌표 = 월드 좌표 + (2.0, 0.5). (maps/tb3_world_cartographer 기준)
# 순서: 1) Nav2 노드 상태  2) 실제 위치로 /initialpose 설정  3) AMCL 추정 vs 실제  4) 목표 전송·결과  5) 도착 오차
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/env.sh"
LOG_DIR="$SCRIPT_DIR/../logs"; mkdir -p "$LOG_DIR"
OUT="$LOG_DIR/nav_test_$(date +%Y%m%d_%H%M%S).txt"
GX="${1:-3.6}"; GY="${2:-0.0}"; GYAW_DEG="${3:-0}"; PX="${4:-}"; PY="${5:-}"
GYAW=$(python3 -c "import math; print(math.radians($GYAW_DEG))")
GQZ=$(python3 -c "import math; print(math.sin($GYAW/2))"); GQW=$(python3 -c "import math; print(math.cos($GYAW/2))")
MAP_OFFSET_X=2.0; MAP_OFFSET_Y=0.5

truth() {  # prints "x y yaw" in MAP frame from Gazebo ground truth
  gz model -m burger -p 2>/dev/null | python3 -c "
import sys, re
nums = [l for l in sys.stdin.read().splitlines() if re.match(r'\s*\[', l)]
xyz = [float(v) for v in re.findall(r'-?\d+\.\d+(?:e-?\d+)?', nums[0])]
rpy = [float(v) for v in re.findall(r'-?\d+\.\d+(?:e-?\d+)?', nums[1])]
print(f'{xyz[0]+$MAP_OFFSET_X:.3f} {xyz[1]+$MAP_OFFSET_Y:.3f} {rpy[2]:.3f}')
"
}

amcl() {  # prints "x y yaw" from one /amcl_pose message
  timeout 8 ros2 topic echo --once /amcl_pose geometry_msgs/msg/PoseWithCovarianceStamped 2>/dev/null | python3 -c "
import sys, yaml, math
d = yaml.safe_load(sys.stdin.read().split('---')[0])
p = d['pose']['pose']; q = p['orientation']
yaw = math.atan2(2*(q['w']*q['z']+q['x']*q['y']), 1-2*(q['y']**2+q['z']**2))
print(f\"{p['position']['x']:.3f} {p['position']['y']:.3f} {yaw:.3f}\")
"
}

err() {  # "x1 y1 yaw1" "x2 y2 yaw2" -> position / yaw error
  python3 -c "
import math
a=[float(v) for v in '$1'.split()]; b=[float(v) for v in '$2'.split()]
dy=(a[2]-b[2]+math.pi)%(2*math.pi)-math.pi if len(a)>2 and len(b)>2 else 0.0
print(f'position error {math.hypot(a[0]-b[0],a[1]-b[1]):.3f} m, yaw error {math.degrees(abs(dy)):.1f} deg')
"
}

{
  echo "=== $(date) ===  goal(map)=($GX, $GY, ${GYAW_DEG} deg)${PX:+  pillar=($PX, $PY)}"

  echo; echo "### 1) wait for Nav2 bringup (bt_navigator active, max 180 s)"
  T_WAIT=0
  until timeout 5 ros2 lifecycle get /bt_navigator 2>/dev/null | grep -q "^active"; do
    sleep 5; T_WAIT=$((T_WAIT+5))
    if [ $T_WAIT -ge 180 ]; then echo "bt_navigator not active after 180 s -> check logs/nav_launch_*.txt"; break; fi
  done
  echo "waited ${T_WAIT} s"
  for n in map_server amcl planner_server controller_server bt_navigator; do
    printf "%-20s " "$n"; timeout 5 ros2 lifecycle get /$n 2>&1 | head -n1
  done

  echo; echo "### 2) set /initialpose from ground truth"
  T0="$(truth)"
  if [ -z "$T0" ]; then echo "ground truth unavailable (gz model failed)"; exit 1; fi
  echo "ground truth (map frame): $T0"
  read X Y YAW <<< "$T0"
  QZ=$(python3 -c "import math; print(math.sin($YAW/2))"); QW=$(python3 -c "import math; print(math.cos($YAW/2))")
  ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
    "{header: {frame_id: map}, pose: {pose: {position: {x: $X, y: $Y}, orientation: {z: $QZ, w: $QW}}, covariance: [0.25,0,0,0,0,0, 0,0.25,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0.0685]}}" > /dev/null
  sleep 3

  echo; echo "### 3) AMCL estimate vs ground truth (before moving)"
  A0="$(amcl)"; echo "amcl:  ${A0:-none}"; echo "truth: $(truth)"
  [ -n "$A0" ] && err "$A0" "$(truth)"

  echo; echo "### 4) send goal /navigate_to_pose"
  START=$(date +%s.%N)
  timeout 180 ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
    "{pose: {header: {frame_id: map}, pose: {position: {x: $GX, y: $GY}, orientation: {z: $GQZ, w: $GQW}}}}" 2>&1 \
    | grep -E "accepted|rejected|status|SUCCEEDED|ABORTED|CANCELED|error_code|error_msg" | head -n 10
  END=$(date +%s.%N)
  echo "wall time: $(python3 -c "print(f'{$END-$START:.1f} s')")"
  sleep 2

  echo; echo "### 5) arrival"
  T1="$(truth)"; A1="$(amcl)"
  echo "goal:  $GX $GY $GYAW"; echo "truth: $T1"; echo "amcl:  ${A1:-none}"
  echo -n "truth vs goal: "; err "$GX $GY $GYAW" "$T1"
  if [ -n "$PX" ]; then
    python3 -c "
import math
t=[float(v) for v in '$T1'.split()]
print(f'robot center to pillar center: {math.hypot(t[0]-$PX, t[1]-$PY):.3f} m')"
  fi
  [ -n "$A1" ] && { echo -n "amcl vs truth: "; err "$A1" "$T1"; }
  echo "=== done ==="
} > "$OUT" 2>&1
echo "saved: $OUT"
