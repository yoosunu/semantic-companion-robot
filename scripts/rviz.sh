#!/bin/bash
# [VM] 실물 로봇을 RViz 로 보기. Pi 에서 nav(또는 robot) 가 돌고 있어야 함.
#   ./scripts/rviz.sh
# 조건 (11-05): UTM 네트워크 = 브리지됨(en0) → VM 이 집 Wi-Fi(192.168.123.x) 에 직접 붙음. ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET (양쪽).
# 시작 화면: 위에서 내려다보기, scale 180, x 0, y 0 (11-07 Jasper)
# 화면 설정: config/robot.rviz (지도·스캔·로봇·경로·비용 지도·탐색 경계·충돌 감시 상자·odom 화살표).
#   주황 화살표 자취 = 받은 순간의 odom 위치 (최근 200개). SLAM 이 위치를 고치면 로봇 모델이 자취에서 "툭" 튐 = 고친 양.
# 창이 안 뜨면 (SSH 로 실행했을 때): DISPLAY=:0 ./scripts/rviz.sh  → VM 화면에 뜸
set -e
REPO="$(cd "$(dirname "$0")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
pkill -f "rviz2 -d" 2>/dev/null && sleep 1   # 떠 있던 RViz 는 끄고 새로 (지도 Reset 대신). 닫을 때 저장 안 함 → 설정 파일 그대로
exec rviz2 -d "$REPO/config/robot.rviz"
