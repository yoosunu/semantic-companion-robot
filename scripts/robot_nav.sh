#!/bin/bash
# Nav2 + RViz 실행. robot up / robot up slam 이 nav2 창에서 자동 실행 (robot reload = 재시작).
#
#   ./scripts/robot_nav.sh [맵 이름]   저장된 지도 + AMCL 위치추정 (기본 맵: tb3_world_cartographer)
#   ./scripts/robot_nav.sh --slam      지도 없이 시작, SLAM Toolbox가 그리는 지도 위에서 주행
#
# 설정 = 원본 + 내 설정값 파일(있으면 덮어씀):
#   Nav2: config/nav2_burger_sim.yaml (+ SLAM 모드면 config/nav2_slam_mode.yaml) + config/my_nav2.yaml
#   SLAM: config/slam_toolbox.yaml    + config/my_slam.yaml
# Nav2 원본: TurtleBot3 burger.yaml 사본 (변경 사항은 그 파일 상단에 기록)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$SCRIPT_DIR/.." && pwd)"
source "$SCRIPT_DIR/env.sh"
LOG="$REPO/logs/nav_launch_$(date +%Y%m%d_%H%M%S).txt"
echo "launch log: $LOG"

merge() {  # merge <원본> <내 파일> <결과> : 결과 경로를 stdout 마지막 줄로
  local base="$1" mine="$2" out="$3"
  if [ ! -f "$mine" ]; then echo "$base"; return; fi
  local msg rc
  msg="$(python3 "$SCRIPT_DIR/merge_params.py" "$base" "$mine" "$out" 2>&1)"; rc=$?
  echo "$msg" | tee -a "$LOG" >&2
  [ $rc -eq 0 ] || { echo "$mine 처리 실패 → 위 메시지 확인" >&2; exit 1; }
  echo "$out"
}

NAV_BASE="$REPO/config/nav2_burger_sim.yaml"
if [ "$1" = "--slam" ]; then   # SLAM 모드 전용 차이(config/nav2_slam_mode.yaml)를 먼저 덮고, 그 위에 내 설정값
  NAV_BASE="$(merge "$NAV_BASE" "$REPO/config/nav2_slam_mode.yaml" "$REPO/logs/nav2_params_slam_base.yaml")" || exit 1
fi
NAV_PARAMS="$(merge "$NAV_BASE" "$REPO/config/my_nav2.yaml" "$REPO/logs/nav2_params_effective.yaml")" || exit 1

if [ "$1" = "--slam" ]; then
  ros2 pkg prefix slam_toolbox >/dev/null 2>&1 || { echo "slam_toolbox 없음 → sudo apt install ros-jazzy-slam-toolbox"; exit 1; }
  SLAM_PARAMS="$(merge "$REPO/config/slam_toolbox.yaml" "$REPO/config/my_slam.yaml" "$REPO/logs/slam_params_effective.yaml")" || exit 1
  echo "[mode] SLAM: 빈 지도에서 시작" | tee -a "$LOG"
  ros2 launch "$SCRIPT_DIR/slam_nav.launch.py" nav_params:="$NAV_PARAMS" slam_params:="$SLAM_PARAMS" 2>&1 | tee -a "$LOG"
else
  MAP="$REPO/maps/${1:-tb3_world_cartographer}.yaml"
  [ -f "$MAP" ] || { echo "map not found: $MAP"; exit 1; }
  echo "[mode] 저장된 지도: $MAP" | tee -a "$LOG"
  ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:="$MAP" params_file:="$NAV_PARAMS" 2>&1 | tee -a "$LOG"
fi
