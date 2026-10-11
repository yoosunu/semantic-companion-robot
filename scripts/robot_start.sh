#!/bin/bash
# Gazebo(turtlebot3_world) + scr_burger 를 tmux 세션 "robot"에서 실행.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SESSION="robot"

if tmux has-session -t "$SESSION" 2>/dev/null; then
    echo "Robot session already running."
    tmux attach -t "$SESSION"
    exit 0
fi

tmux new-session -d -s "$SESSION" -n gazebo
tmux send-keys -t "$SESSION:gazebo" \
"source $SCRIPT_DIR/env.sh && ros2 launch scr_gazebo sim_world.launch.py" C-m

tmux attach -t "$SESSION"
