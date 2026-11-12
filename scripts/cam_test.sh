#!/bin/bash
# [Pi] 카메라 시험: 640x480·30fps 와 320x240·10fps 를 차례로 켜서 → 초당 장수 + 사진 (화각이 같은지 비교)
#   bash ~/semantic-companion-robot/scripts/cam_test.sh      → ~/cam_640x480.png, ~/cam_320x240.png
R=~/semantic-companion-robot
source /opt/ros/jazzy/setup.bash; source ~/cam_ws/install/setup.bash
pkill -f "camera_ros/camera_node" 2>/dev/null; sleep 1
for m in "640 480 30" "320 240 10"; do
  set -- $m
  echo "=== $1x$2 @ $3 fps"
  bash $R/scripts/cam_run.sh $1 $2 $3 > /tmp/cam_node.log 2>&1 &
  sleep 6
  grep -iE "error|fail|warn|mode|configur" /tmp/cam_node.log | tail -4
  echo "--- 초당 장수 (5 s)"; timeout 5 ros2 topic hz /camera/image_raw 2>/dev/null | grep average | tail -1
  python3 $R/scripts/cam_snap.py ~/cam_$1x$2.png
  pkill -f "camera_ros/camera_node" 2>/dev/null; sleep 2   # ros2 run 만 죽이면 노드가 카메라를 붙잡음 (11-12)
done
