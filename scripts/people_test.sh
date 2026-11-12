#!/bin/bash
# [Pi] /people 시험 (P1): look_test(브리지·카메라 640·고개) + face_perception(검출·추적). 주행 안 함.
#   [DET_W=640] bash ~/semantic-companion-robot/scripts/people_test.sh      끄기: bash ~/semantic-companion-robot/scripts/stop_all.sh
R=~/semantic-companion-robot
CAM="640 480 10" bash $R/scripts/look_test.sh > /dev/null
tmux new -d -s face "bash -ic 'source ~/scr_ws/install/setup.bash; ~/yolo_venv/bin/python -u $R/scripts/face_perception.py --ros-args -p det_w:=${DET_W:-640} 2> >(grep -v WARN >&2); exec bash'"
sleep 6
echo "--- face_perception"; tmux capture-pane -J -pt face | grep -v '^$' | tail -4
echo "보기: tmux attach -t face  (나오기 Ctrl+b → d)   토픽: ros2 topic echo /people --once"
