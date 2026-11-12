#!/bin/bash
# [Pi] 고개 시험 (11-12 v2): 브리지(+LiDAR) → 카메라 640x480·10fps → face_perception(보기) → look(고개). 주행 안 함.
#   bash ~/semantic-companion-robot/scripts/look_test.sh      끄기: bash ~/semantic-companion-robot/scripts/stop_all.sh
#   옵션: CAM="320 240 10" (카메라), DET_W=320 (검출 폭), LOOK_ARGS="-p doze_after:=20.0 -p sleep_after:=30.0" (빨리 졸기 시험)
R=~/semantic-companion-robot
bash $R/scripts/stop_all.sh quiet; pkill -f "camera_ros/camera_node" 2>/dev/null   # 남은 카메라 노드 (카메라는 한 번에 하나만)
tmux new -d -s robot "bash -ic 'ros2 launch scr_bringup robot.launch.py ugv_port:=/dev/ttyAMA0; exec bash'"
sleep 6
# 카메라: 640 (11-12 실측: 바닥에서 먼 얼굴은 640 검출이어야 잡힘. 검출은 face_perception 이 det_w 로)
tmux new -d -s cam "bash $R/scripts/cam_run.sh ${CAM:-640 480 10}; exec bash"
sleep 5
# 보기: OpenCV 4.8+ 필요 → yolo_venv 파이썬. scr_interfaces 는 ~/scr_ws 에서 (bash -ic 가 .bashrc 로 source)
tmux new -d -s face "bash -ic 'source ~/scr_ws/install/setup.bash; ~/yolo_venv/bin/python -u $R/scripts/face_perception.py --ros-args -p det_w:=${DET_W:-640} 2> >(grep -v WARN >&2); exec bash'"
tmux new -d -s look "bash -ic 'source ~/scr_ws/install/setup.bash; python3 -u $R/scripts/look_at_person.py --ros-args ${LOOK_ARGS}; exec bash'"
sleep 8
echo "--- 보기";  tmux capture-pane -J -pt face | grep -v '^$' | tail -2
echo "--- 고개";  tmux capture-pane -J -pt look | grep -v '^$' | tail -3
echo "로그: tmux capture-pane -J -pt look -S -300 | grep LOOK | tail -20"
