#!/bin/bash
# 현재 SLAM 맵(/map)을 maps/<이름>.{pgm,yaml} 로 저장한다.
# 사용: ./scripts/save_map.sh <이름>     예) ./scripts/save_map.sh tb3_world_cartographer
#
# map_subscribe_transient_local:=false
#   map_saver는 기본적으로 transient_local(늦게 붙어도 마지막 맵 수신)로 구독하지만,
#   Cartographer occupancy grid 노드는 /map을 volatile로 publish한다. ROS 2에서
#   transient_local 구독 + volatile 발행은 호환되지 않아 연결이 안 됨 → 구독을 volatile로 맞춘다.
#   (SLAM Toolbox 등 transient_local로 발행하는 노드라면 이 옵션 없이도 동작)
# save_map_timeout:=10.0 : 맵 수신 대기 시간(기본 2초)을 늘림.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/env.sh"
NAME="${1:?usage: save_map.sh <name>}"
mkdir -p "$SCRIPT_DIR/../maps"
ros2 run nav2_map_server map_saver_cli -f "$SCRIPT_DIR/../maps/$NAME" --ros-args \
  -p use_sim_time:=true \
  -p map_subscribe_transient_local:=false \
  -p save_map_timeout:=10.0
