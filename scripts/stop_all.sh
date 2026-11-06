#!/bin/bash
# [Pi] 로봇 쪽 전부 멈추기 (탐색기 → 심박 → nav/robot). Ctrl+C 로 끄므로 브리지가 정지 명령을 보내고 보고서가 만들어짐.
for s in explore hb whisker nav robot shuttle; do tmux send-keys -t $s C-c 2>/dev/null; done
sleep 5
for s in explore hb whisker nav robot shuttle; do tmux kill-session -t $s 2>/dev/null; done
[ "$1" = "quiet" ] || { tmux ls 2>/dev/null || echo "전부 꺼짐"; }
