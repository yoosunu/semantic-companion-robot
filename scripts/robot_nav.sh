#!/bin/bash
# 저장된 맵으로 Nav2(AMCL 위치추정 + 경로계획/주행) + RViz 실행.
# 사용: robot-start 로 Gazebo를 띄운 뒤  ./scripts/robot_nav.sh [맵 이름]   (기본: tb3_world_cartographer)
# SLAM(robot-slam)과 동시에 실행하지 말 것: 둘 다 map->odom TF를 발행해 충돌한다.
#
# 파라미터: config/nav2_burger_sim.yaml (TurtleBot3 burger.yaml 사본 + 변경 사항은 파일 상단에 기록).
#   - enable_stamped_cmd_vel: true → Gazebo bridge의 TwistStamped와 일치 (확인 완료)
#   - 알려진 위험: max_vel_x 0.3 (Burger 최대 0.22 초과), velocity_smoother max_accel 2.5 m/s^2
#     → 기준선 주행 결과를 보고, 문제가 확인되면 repo로 파라미터를 복사해 조정한다.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/env.sh"
MAP_NAME="${1:-tb3_world_cartographer}"
MAP="$SCRIPT_DIR/../maps/$MAP_NAME.yaml"
[ -f "$MAP" ] || { echo "map not found: $MAP"; exit 1; }
LOG="$SCRIPT_DIR/../logs/nav_launch_$(date +%Y%m%d_%H%M%S).txt"
echo "launch log: $LOG"
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:="$(realpath "$MAP")" params_file:="$(realpath "$SCRIPT_DIR/../config/nav2_burger_sim.yaml")" 2>&1 | tee "$LOG"
