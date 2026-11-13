#!/bin/bash
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/env.sh"
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True
