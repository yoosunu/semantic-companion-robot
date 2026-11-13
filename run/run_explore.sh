#!/bin/bash
# [Pi] 한 줄로 탐색 시작: 다 끄기 → 최신 받기 → nav(브리지·LiDAR·SLAM·Nav2·Whisker) → 심박 → 우리 탐색기 → 상태
#   ~/semantic-companion-robot/run/run_explore.sh [지도이름=room]
# 로봇은 먼저 "집" 자리에 (지도 기준이 됨). 보기: tmux attach -t explore (나오기 Ctrl+b → d). 멈추기: run/stop_all.sh
NAME="${1:-room}"
REPO=~/semantic-companion-robot
bash "$REPO/run/stop_all.sh" quiet
cd "$REPO" && git pull --ff-only -q || { echo "git pull 실패"; exit 1; }
LOGD=~/scr_logs/$(date +%Y-%m-%d); mkdir -p "$LOGD"; STAMP=$(date +%H%M)
echo "nav 켜는 중 (25초)… 기록: $LOGD/{nav,explore}_$STAMP.log"
tmux new -d -s nav "bash -ic 'ros2 launch scr_bringup nav.launch.py ugv_port:=/dev/ttyAMA0 2>&1 | tee -ia $LOGD/nav_$STAMP.log; exec bash'"
sleep 25
# 11-11: ros2 node list 는 노드가 30개 가까우면 다 못 찾음 (위스커·브리지가 멀쩡히 서로 통하는데 목록엔 없음, 데몬 없이도 같음)
#   → 프로세스로 셈 (위스커가 죽었으면 프로세스도 없음)
n=$(pgrep -fc "nodes/navigation/whisker.py")
echo "Whisker 노드: $n 개 (1 이어야 함)"
[ "$n" = "1" ] || { echo "→ 이상함. tmux capture-pane -J -pt nav | tail -30 로 확인"; exit 1; }
tmux new -d -s hb "bash -ic 'python3 $REPO/nodes/system/heartbeat.py --period 30; exec bash'"
tmux new -d -s explore "bash -ic 'python3 -u $REPO/nodes/navigation/scr_explore.py --name $NAME 2>&1 | tee -ia $LOGD/explore_$STAMP.log; exec bash'"
sleep 8
tmux capture-pane -J -pt explore | grep EXPLORE | tail -3
echo "보기: tmux attach -t explore  |  Whisker: tmux capture-pane -J -pt nav | grep -E 'SENSE|WHISKER' | tail  |  멈추기: bash $REPO/run/stop_all.sh"
