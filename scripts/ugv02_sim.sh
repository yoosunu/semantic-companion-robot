#!/bin/bash
# "UGV02 모드" 시뮬: Gazebo TB3를 UGV02라고 가정하고, 내 ugv02_bridge를 거쳐서만 움직이게 한다.
#
#   /cmd_vel ─> ugv02_bridge ─JSON─> /tmp/ugv02_bridge ══(가상 시리얼, socat)══ /tmp/ugv02_esp32 ─> fake_esp32 ─/sim/cmd_vel─> Gazebo
#   /odom, TF <─ ugv02_bridge <─ T:1001 JSON (바퀴 속도, 전압) ─────────────────── fake_esp32 <─/sim/odom── Gazebo
#
# Gazebo의 cmd_vel/odom/tf는 /sim/* 로 숨겨진다 → 브리지를 안 켜면 teleop을 눌러도 안 움직이고, /odom 도 없는 게 정상.
# 종료: robot-stop (평소와 같음)
# 화면 없이(Gazebo 창 끄고) 실행: GUI=false ./scripts/ugv02_sim.sh  → 확인은 esp32 창 로그 / ros2 topic echo
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SESSION="robot"

command -v socat >/dev/null || { echo "socat 없음: sudo apt install socat"; exit 1; }
python3 -c "import serial" 2>/dev/null || { echo "pyserial 없음: sudo apt install python3-serial"; exit 1; }

if tmux has-session -t "$SESSION" 2>/dev/null; then
    echo "Robot session already running. 먼저 robot-stop."
    exit 1
fi

tmux new-session -d -s "$SESSION" -n gazebo
tmux send-keys -t "$SESSION:gazebo" \
"source $SCRIPT_DIR/env.sh && ros2 launch scr_gazebo sim_world.launch.py ugv02_mode:=true gui:=${GUI:-true}" C-m

# 가상 시리얼 케이블: 양 끝이 /tmp/ugv02_bridge (브리지 쪽) 와 /tmp/ugv02_esp32 (가짜 ESP32 쪽)
tmux new-window -t "$SESSION" -n serial
tmux send-keys -t "$SESSION:serial" \
"socat -d -d pty,raw,echo=0,link=/tmp/ugv02_bridge pty,raw,echo=0,link=/tmp/ugv02_esp32" C-m

tmux new-window -t "$SESSION" -n esp32
tmux send-keys -t "$SESSION:esp32" \
"until [ -e /tmp/ugv02_esp32 ]; do sleep 0.2; done; source $SCRIPT_DIR/env.sh && python3 $SCRIPT_DIR/fake_esp32.py --port /tmp/ugv02_esp32" C-m

tmux select-window -t "$SESSION:esp32"
tmux attach -t "$SESSION"
