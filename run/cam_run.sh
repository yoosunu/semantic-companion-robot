#!/bin/bash
# [Pi] 카메라 노드 켜기: bash cam_run.sh [폭] [높이] [fps]   (기본 320 240 10)
#   11-12: 얼굴·YOLO 둘 다 320 폭으로 줄여 돌리니 640x480·30fps 는 낭비 → 320x240·10fps (화각 같은지 cam_test 로 확인)
#   카메라(/dev/media*)는 video 그룹. 지금 셸에 그룹이 없으면(tmux 가 옛 권한) sg video 로 다시 실행
W=${1:-320}; H=${2:-240}; F=${3:-10}
if ! id -nG | grep -qw video; then exec sg video -c "bash $0 $W $H $F"; fi
source /opt/ros/jazzy/setup.bash; source ~/cam_ws/install/setup.bash
D=$((1000000 / F))                      # 한 장 시간 µs (최소=최대 → 고정 fps)
exec ros2 run camera_ros camera_node --ros-args -p width:=$W -p height:=$H -p format:=RGB888 \
  -p FrameDurationLimits:="[$D,$D]"
