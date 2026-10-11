#!/bin/bash
# 현재 SLAM 맵(/map)을 maps/<이름>.{pgm,yaml} 로 저장한다.
# 사용: ./scripts/save_map.sh <이름>     예) ./scripts/save_map.sh tb3_world_cartographer
#
# 첫 시도(기본값)는 "Failed to spin map subscription"으로 실패, 아래 옵션으로 성공.
# QoS 불일치를 의심했으나 확인 결과 Cartographer /map 발행 QoS는 TRANSIENT_LOCAL로 기본 구독과 호환됨
# (docs/debugging/001/logs/map_qos_20261011.txt). 실제 원인은 기본 대기 시간(2초) 안에 맵을 못 받은 것으로 추정.
# 검증된 조합을 그대로 유지한다:
#   save_map_timeout:=10.0              맵 수신 대기 시간 2초 → 10초
#   map_subscribe_transient_local:=false volatile 구독 (어떤 발행 QoS와도 호환)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/env.sh"
NAME="${1:?usage: save_map.sh <name>}"
mkdir -p "$SCRIPT_DIR/../maps"
ros2 run nav2_map_server map_saver_cli -f "$SCRIPT_DIR/../maps/$NAME" --ros-args \
  -p use_sim_time:=true \
  -p map_subscribe_transient_local:=false \
  -p save_map_timeout:=10.0
