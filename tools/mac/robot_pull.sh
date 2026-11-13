#!/bin/bash
# [Mac] Pi 의 주행 보고서와 ROS 로그를 가져오고, 가장 최근 보고서를 연다.
#   robot-pull                                  (alias, ~/.zshrc)
#   PI_HOST=jasper@192.168.x.x robot-pull       (scr-pi.local 이 안 찾아질 때)
# 보고서는 Pi 가 주행이 끝날 때 스스로 만든다 (nav.launch.py 종료 시 → run_report.py). 여기선 가져오기만.
# 가져온 것은 logs/ 아래 → .gitignore 대상 (커밋 안 됨).
set -euo pipefail
REPO="$(cd "$(dirname "$0")/../.." && pwd)"
HOST="${PI_HOST:-jasper@192.168.123.114}"   # 11-08: scr-pi.local 이 자주 못 찾아져서 IP 기본
SSH="ssh -o ConnectTimeout=8"

ping -c 1 -t 3 "${HOST#*@}" >/dev/null 2>&1 || true   # Pi Wi-Fi 절전 깨우기 (첫 접속 타임아웃 방지)
mkdir -p "$REPO/logs/reports" "$REPO/logs/ros_log"
rsync -a -e "$SSH" "$HOST:scr_logs/reports/" "$REPO/logs/reports/"
rsync -a -e "$SSH" --include='*.log' --exclude='*' "$HOST:.ros/log/" "$REPO/logs/ros_log/"
# 심박(날짜별 heartbeat.jsonl), nav·탐색기 전체 출력(11-07~), 저장한 지도도 같이
rsync -am -e "$SSH" --include='*/' --include='heartbeat.jsonl' --include='nav_*.log' --include='explore_*.log' --exclude='*' "$HOST:scr_logs/" "$REPO/logs/scr_logs/"
rsync -a -e "$SSH" "$HOST:maps/" "$REPO/logs/maps/" 2>/dev/null || true
rsync -a -e "$SSH" "$HOST:scr_logs/imu_lab/" "$REPO/logs/imu_lab/" 2>/dev/null || true   # IMU 실험 (11-06)
rsync -a -e "$SSH" "$HOST:scr_logs/whisker/" "$REPO/logs/whisker/" 2>/dev/null || true   # Whisker 접촉 기록 (11-06)

latest=$(ls -t "$REPO"/logs/reports/*.md 2>/dev/null | head -1 || true)
if [ -n "$latest" ]; then
  echo "최신 보고서: $latest"
  open "$latest"
else
  echo "보고서 없음 (Pi 의 ~/scr_logs/reports 가 비어 있음)"
fi
