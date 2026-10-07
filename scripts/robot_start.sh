#!/bin/bash

SESSION="robot"

if tmux has-session -t "$SESSION" 2>/dev/null; then
    echo "Robot session already running."
    tmux attach -t "$SESSION"
    exit 0
fi

tmux new-session -d -s "$SESSION" -n gazebo

tmux send-keys -t "$SESSION:gazebo" \
"source /opt/ros/jazzy/setup.bash && \
source ~/turtlebot3_ws/install/setup.bash && \
export TURTLEBOT3_MODEL=burger && \
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py" C-m

tmux attach -t "$SESSION"
