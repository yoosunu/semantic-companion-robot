#!/bin/bash
# [Pi] 얼굴 인식 실험실: 카메라가 없으면 켜고 face_lab.py 실행
#   bash face_lab.sh enroll jasper   |   bash face_lab.sh watch [--stream] [--det-w 640 --min-px 0]   |   bash face_lab.sh forget jasper
#   고개랑 같이: CAM="640 480 10" bash look_test.sh 로 먼저 켜 두면 그 카메라를 같이 씀 (안 끄고 안 다시 켬)
#   그림: ~/face_lab/ (Mac 으로: scp 'jasper@scr-pi.local:~/face_lab/*' ~/Downloads/face_lab/)
#   카메라 640x480·10fps: 검출은 320 으로 줄여서, 얼굴 자르기는 원본에서 (인식은 선명할수록 좋음, CPU 차이 작음)
R=~/semantic-companion-robot; MINE=0
if [ "$1" != forget ] && ! pgrep -f "camera_ros/camera_node" >/dev/null; then
  tmux new -d -s cam "bash $R/scripts/cam_run.sh 640 480 10; exec bash"; sleep 6; MINE=1
fi
source /opt/ros/jazzy/setup.bash
~/yolo_venv/bin/python -u $R/scripts/face_lab.py "$@" 2> >(grep -v "WARN" >&2)
[ $MINE = 1 ] && { tmux kill-session -t cam 2>/dev/null; pkill -f "camera_ros/camera_node"; }   # 내가 켠 카메라만 끔
