#!/bin/bash

SESSION="robot"

if ! tmux has-session -t "$SESSION" 2>/dev/null; then
    echo "Robot session is not running."
    exit 0
fi

echo "Stopping robot environment..."

# 각 window에 Ctrl+C 전달
tmux send-keys -t "$SESSION:teleop" C-c 2>/dev/null
sleep 1

tmux send-keys -t "$SESSION:rviz" C-c 2>/dev/null
sleep 1

tmux send-keys -t "$SESSION:gazebo" C-c 2>/dev/null
sleep 3

# 남은 tmux session 종료
tmux kill-session -t "$SESSION"

echo "Robot environment stopped."
