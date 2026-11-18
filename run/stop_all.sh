#!/bin/bash
# [Pi] 로봇 쪽 전부 멈추기 (탐색기 → 심박 → nav/robot). Ctrl+C 로 끄므로 브리지가 정지 명령을 보내고 보고서가 만들어짐.
for s in exec oled yolo floor inspect face look cam explore hb whisker nav robot shuttle; do tmux send-keys -t $s C-c 2>/dev/null; done
# Nav2 전체가 꺼지고 보고서가 만들어질 때까지 기다림 (11-08: 5 s 만 기다려 보고서가 안 생김). 최대 25 s
for i in $(seq 1 25); do pgrep -f "ros2 launch|run_report.py" >/dev/null || break; sleep 1; done
for s in exec oled yolo floor inspect face look cam explore hb whisker nav robot shuttle; do tmux kill-session -t $s 2>/dev/null; done
# 11-18: tmux 를 닫아도 떨어져 나와 살아남은 노드가 있었음 (옛 robot_state_publisher 가 옛 URDF TF 를 계속 냄 → laser_x 0).
#   남은 것: 먼저 SIGINT (브리지가 정지 명령 보낼 기회), 2 s 뒤에도 있으면 SIGTERM. 이름을 보여줌
LEFT="robot_state_publisher|ugv02_bridge|sllidar_node|scan_to_scan_filter_chain|nav2_|slam_toolbox|camera_node|component_container|ros2 launch|lab\.py|nodes/.*\.py"
if pgrep -f "$LEFT" >/dev/null; then
  echo "남은 프로세스 정리:"; pgrep -af "$LEFT" | grep -v pgrep | cut -c1-110
  pkill -INT -f "$LEFT"; sleep 2
  pkill -TERM -f "$LEFT" 2>/dev/null; sleep 1
fi
[ "$1" = "quiet" ] || { tmux ls 2>/dev/null || echo "전부 꺼짐"; pgrep -af "$LEFT" | grep -v pgrep || true; }
