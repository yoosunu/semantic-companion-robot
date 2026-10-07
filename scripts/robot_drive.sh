#!/bin/bash

source /opt/ros/jazzy/setup.bash
source ~/turtlebot3_ws/install/setup.bash
export TURTLEBOT3_MODEL=burger

python3 ~/robot_project/scripts/safe_teleop.py
