#!/bin/bash
# [Pi] "나 쳐다보기" 시험: 브리지(+LiDAR) → 카메라 → YOLO 고개. 주행은 안 함 (Nav2·탐색기 없음).
#   bash ~/semantic-companion-robot/scripts/look_test.sh      끄기: bash ~/semantic-companion-robot/scripts/stop_all.sh
R=~/semantic-companion-robot
bash $R/scripts/stop_all.sh quiet; pkill -f "camera_ros/camera_node" 2>/dev/null   # 남은 카메라 노드 (카메라는 한 번에 하나만)
tmux new -d -s robot "bash -ic 'ros2 launch scr_bringup robot.launch.py ugv_port:=/dev/ttyAMA0; exec bash'"
sleep 6
# 카메라(/dev/media*)는 video 그룹 권한. tmux 서버가 그룹 추가 전에 켜졌으면 새 창도 옛 권한 → sg video 로 확실히 (11-12)
tmux new -d -s cam "sg video -c 'bash -ic \"ros2 run camera_ros camera_node --ros-args -p width:=640 -p height:=480 -p format:=RGB888\"'; exec bash"
sleep 5
tmux new -d -s look "bash -ic '~/yolo_venv/bin/python -u $R/scripts/look_at_person.py; exec bash'"
sleep 8
echo "--- 고개 노드 (Ctrl+C 로 이 화면만 나감, 노드는 계속)"; tmux capture-pane -J -pt look | tail -5
echo "보기: tmux attach -t look  (나오기 Ctrl+b → d)"
