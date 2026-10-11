#!/bin/bash
# 공통 ROS 2 환경. 다른 스크립트에서 `source "$SCRIPT_DIR/env.sh"` 로 사용.
source /opt/ros/jazzy/setup.bash
[ -f ~/turtlebot3_ws/install/setup.bash ] && source ~/turtlebot3_ws/install/setup.bash
[ -f ~/scr_ws/install/setup.bash ]        && source ~/scr_ws/install/setup.bash
export TURTLEBOT3_MODEL=burger
