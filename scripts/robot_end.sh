#!/bin/bash
# [Mac] 주행 끝: Pi 전부 끄기(보고서 생길 때까지 기다림) → 로그·보고서 가져오기 → 최신 보고서 열기. 한 줄로.
#   bash scripts/robot_end.sh            (Pi 주소 기본 jasper@192.168.123.114)
#   PI_HOST=jasper@scr-pi.local bash scripts/robot_end.sh
set -uo pipefail
export PI_HOST="${PI_HOST:-jasper@192.168.123.114}"
REPO="$(cd "$(dirname "$0")/.." && pwd)"
echo "Pi 끄는 중 (최대 30초)…"
ssh -o ConnectTimeout=8 "$PI_HOST" 'bash ~/semantic-companion-robot/scripts/stop_all.sh' || echo "→ Pi 에 못 붙음 (주소·전원 확인)"
bash "$REPO/scripts/robot_pull.sh"
