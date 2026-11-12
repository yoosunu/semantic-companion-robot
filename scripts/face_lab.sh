#!/bin/bash
# [Pi] 얼굴 인식 실험실: 카메라만 켜고 face_lab.py 실행 (주행·고개 없음)
#   bash face_lab.sh enroll jasper   |   bash face_lab.sh watch   |   bash face_lab.sh forget jasper
#   그림: ~/face_lab/ (Mac 으로: scp 'jasper@scr-pi.local:~/face_lab/*.jpg' ~/Downloads/face_lab/)
#   카메라 640x480·10fps: 검출은 320 으로 줄여서, 얼굴 자르기는 원본에서 (인식은 선명할수록 좋음, CPU 차이 작음)
R=~/semantic-companion-robot
if [ "$1" != forget ]; then
  bash $R/scripts/stop_all.sh quiet; pkill -f "camera_ros/camera_node" 2>/dev/null; sleep 1
  tmux new -d -s cam "bash $R/scripts/cam_run.sh 640 480 10; exec bash"; sleep 6
fi
source /opt/ros/jazzy/setup.bash
~/yolo_venv/bin/python -u $R/scripts/face_lab.py "$@" 2> >(grep -v "WARN" >&2)
[ "$1" = enroll ] && { tmux kill-session -t cam 2>/dev/null; pkill -f "camera_ros/camera_node"; }
