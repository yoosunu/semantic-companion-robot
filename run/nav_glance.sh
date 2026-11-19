#!/bin/bash
# [Pi] 11-19 비전 바닥 위험물 시험: nav (브리지·LiDAR·SLAM·Nav2·Whisker) + 카메라 + 고개 + 사령관 + 바닥 감지 + 힐끗 + floor_hazard.
#   탐색기 없음 → 목표는 RViz "2D Goal Pose" 로 직접. 얼굴 인식 안 켬 (CPU).
#   졸기 1 시간 뒤 (11-20: 600 s 면 시험 중 잠듦 → 힐끗 안 받음)
#   bash ~/semantic-companion-robot/run/nav_glance.sh        끄기: bash ~/semantic-companion-robot/run/stop_all.sh
#   로그: tmux capture-pane -J -pt hazard -S -200 | grep HAZARD  /  tmux capture-pane -J -pt inspect -S -300 | grep GLANCE
#   표시 지우기: ros2 topic pub --once /semantic/clear std_msgs/msg/String "{data: all}"
R=~/semantic-companion-robot
bash $R/run/stop_all.sh quiet; pkill -f "camera_ros/camera_node" 2>/dev/null
LOGD=~/scr_logs/$(date +%Y-%m-%d); mkdir -p "$LOGD"; STAMP=$(date +%H%M)
echo "nav 켜는 중 (25초)… 기록: $LOGD/{nav,hazard}_$STAMP.log"
tmux new -d -s nav "bash -ic 'ros2 launch scr_bringup nav.launch.py ugv_port:=/dev/ttyAMA0 2>&1 | tee -ia $LOGD/nav_$STAMP.log; exec bash'"
sleep 25
tmux new -d -s cam "bash $R/run/cam_run.sh ${CAM:-640 480 10}; exec bash"
sleep 5
tmux new -d -s look "bash -ic 'source ~/scr_ws/install/setup.bash; python3 -u $R/nodes/behavior/look_at_person.py --ros-args ${LOOK_ARGS:--p scan:=false}; exec bash'"
tmux new -d -s exec "bash -ic 'source ~/scr_ws/install/setup.bash; python3 -u $R/nodes/behavior/executive.py --ros-args ${EXEC_ARGS:--p doze_after:=[3600.0,3600.0]}; exec bash'"
tmux new -d -s floor "bash -ic 'python3 -u $R/nodes/perception/floor_perception.py --ros-args -p stream_port:=8085 ${FLOOR_ARGS}; exec bash'"
tmux new -d -s inspect "bash -ic 'source ~/scr_ws/install/setup.bash; ~/yolo_venv/bin/python -u $R/nodes/perception/glance_node.py --ros-args -p stream_port:=8086 -p yoloe_model:=$HOME/models/yoloe-26s-seg-home.onnx ${INSPECT_ARGS}; exec bash'"
tmux new -d -s labels "bash -ic 'source ~/scr_ws/install/setup.bash; python3 -u $R/nodes/perception/semantic_markers.py; exec bash'"   # 11-20 RViz 이름표
tmux new -d -s hazard "bash -ic 'source ~/scr_ws/install/setup.bash; python3 -u $R/nodes/navigation/floor_hazard.py --ros-args ${HAZARD_ARGS} 2>&1 | tee -ia $LOGD/hazard_$STAMP.log; exec bash'"
sleep 10
n=$(pgrep -fc "nodes/navigation/whisker.py"); echo "Whisker 노드: $n 개 (1 이어야 함)"
echo "--- 힐끗";  tmux capture-pane -J -pt inspect | grep -v '^$' | tail -2
echo "--- 위험물"; tmux capture-pane -J -pt hazard | grep -v '^$' | tail -2
echo "VM: ./tools/mac/rviz.sh → 2D Goal Pose 로 목표. 빨간 점 = 위험물 (/semantic/cloud)"
