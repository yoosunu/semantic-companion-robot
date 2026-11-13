#!/bin/bash
# [Pi] 바닥 이상 감지 실험실: 카메라가 없으면 켜고 floor_lab.py --stream (브라우저 http://<Pi IP>:8082)
#   bash floor_lab.sh [--th 3.5] [--horizon 0.5]      끄기: Ctrl+C (내가 켠 카메라도 같이 끔)
#   바닥이 화면 아래쪽에 보여야 함 → 고개 살짝 아래 (브리지 켜져 있을 때):
#     ros2 topic pub --once /pantilt/cmd std_msgs/msg/Float32MultiArray "{data: [0.0, -15.0, 0.0, 0.0]}"
R=~/semantic-companion-robot; MINE=0
if ! pgrep -f "camera_ros/camera_node" >/dev/null; then
  tmux new -d -s cam "bash $R/scripts/cam_run.sh 640 480 10; exec bash"; sleep 6; MINE=1
fi
source /opt/ros/jazzy/setup.bash
trap '[ $MINE = 1 ] && { tmux kill-session -t cam 2>/dev/null; pkill -f "camera_ros/camera_node"; }' EXIT
~/yolo_venv/bin/python -u $R/scripts/floor_lab.py $([ "${STREAM:-1}" = 0 ] || echo --stream) "$@" 2> >(grep -v "WARN" >&2)
