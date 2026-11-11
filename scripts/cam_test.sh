#!/bin/bash
# [Pi] 카메라 ROS 시험: camera_ros 를 640x480 으로 켜고 → 초당 장수 → 사진 한 장 ~/cam_snap.png (Mac 으로 가져와 보기)
#   bash ~/semantic-companion-robot/scripts/cam_test.sh
source /opt/ros/jazzy/setup.bash; source ~/cam_ws/install/setup.bash
ros2 run camera_ros camera_node --ros-args -p width:=640 -p height:=480 -p format:=RGB888 > /tmp/cam_node.log 2>&1 &
P=$!; sleep 6
echo "--- 노드 로그 (마지막 8줄)"; tail -8 /tmp/cam_node.log
echo "--- 초당 장수 (5 s)"; timeout 5 ros2 topic hz /camera/image_raw 2>/dev/null | tail -2
python3 ~/semantic-companion-robot/scripts/cam_snap.py
kill $P 2>/dev/null; pkill -f "camera_ros/camera_node" 2>/dev/null   # ros2 run 만 죽이면 카메라 노드가 남아 카메라를 붙잡음 (11-12)
