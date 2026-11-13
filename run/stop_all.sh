#!/bin/bash
# [Pi] 로봇 쪽 전부 멈추기 (탐색기 → 심박 → nav/robot). Ctrl+C 로 끄므로 브리지가 정지 명령을 보내고 보고서가 만들어짐.
for s in exec oled yolo floor face look cam explore hb whisker nav robot shuttle; do tmux send-keys -t $s C-c 2>/dev/null; done
# Nav2 전체가 꺼지고 보고서가 만들어질 때까지 기다림 (11-08: 5 s 만 기다려 보고서가 안 생김). 최대 25 s
for i in $(seq 1 25); do pgrep -f "ros2 launch|run_report.py" >/dev/null || break; sleep 1; done
for s in exec oled yolo floor face look cam explore hb whisker nav robot shuttle; do tmux kill-session -t $s 2>/dev/null; done
[ "$1" = "quiet" ] || { tmux ls 2>/dev/null || echo "전부 꺼짐"; }
