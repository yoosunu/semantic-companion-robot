#!/bin/bash
# [Pi] YOLO 사물 실험실: 카메라가 없으면 켜고 yolo_lab.py (기본 --stream → 브라우저 http://scr-pi.local:8081)
#   bash yolo_lab.sh [--conf 0.15]      끄기: Ctrl+C (내가 켠 카메라도 같이 끔)
#   고개 돌려 보기 (브리지 켜져 있을 때): ros2 topic pub --once /pantilt/cmd std_msgs/msg/Float32MultiArray "{data: [0.0, 20.0, 0.0, 0.0]}"
R=~/semantic-companion-robot; MINE=0
if ! pgrep -f "camera_ros/camera_node" >/dev/null; then
  tmux new -d -s cam "bash $R/scripts/cam_run.sh 640 480 10; exec bash"; sleep 6; MINE=1
fi
source /opt/ros/jazzy/setup.bash
trap '[ $MINE = 1 ] && { tmux kill-session -t cam 2>/dev/null; pkill -f "camera_ros/camera_node"; }' EXIT
~/yolo_venv/bin/python -u $R/scripts/yolo_lab.py $([ "${STREAM:-1}" = 0 ] || echo --stream) "$@" 2> >(grep -v "WARN" >&2)
