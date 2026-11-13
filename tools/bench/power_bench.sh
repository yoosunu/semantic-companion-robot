#!/bin/bash
# [Pi] 전력·CPU 재기 (11-12): 하나씩 켜면서 Pi 내부 전력과 CPU 평균. 주행 안 함. 약 3분.
#   bash ~/semantic-companion-robot/tools/bench/power_bench.sh [단계당 초=20]
#   Pi 쪽만 (PMIC). 바퀴·서보·LiDAR 는 UGV02 배터리에서 바로 먹어서 여기 안 잡힘 → DC 전력계로 따로.
R=~/semantic-companion-robot; S=${1:-20}; P="python3 $R/tools/bench/power_probe.py $S"
command -v vcgencmd >/dev/null || { echo "vcgencmd 없음 → sudo apt install -y libraspberrypi-bin 후 다시"; exit 1; }
SG=0; vcgencmd pmic_read_adc 2>/dev/null | grep -q volt || SG=1      # 권한 없으면 video 그룹으로
run(){ if [ $SG = 1 ]; then sg video -c "$P '$1'"; else $P "$1"; fi; }
bash $R/run/stop_all.sh quiet; pkill -f "camera_ros/camera_node" 2>/dev/null; sleep 3
run "0 바탕 (ROS 없음)"
tmux new -d -s robot "bash -ic 'ros2 launch scr_bringup robot.launch.py ugv_port:=/dev/ttyAMA0; exec bash'"; sleep 10
run "1 브리지+LiDAR 드라이버"
tmux new -d -s cam "bash $R/run/cam_run.sh 640 480 30; exec bash"; sleep 8
run "2 +카메라 640x480 30fps"
tmux kill-session -t cam; pkill -f "camera_ros/camera_node"; sleep 2
tmux new -d -s cam "bash $R/run/cam_run.sh 320 240 10; exec bash"; sleep 8
run "3 +카메라 320x240 10fps"
tmux new -d -s look "bash -ic '~/yolo_venv/bin/python -u $R/nodes/behavior/look_at_person.py; exec bash'"; sleep 10
read -p ">> 카메라 앞에 앉아서 얼굴 보여주고 Enter " _
run "4 +look 얼굴 있음 (YuNet)"
read -p ">> 카메라에서 안 보이게 비키고 Enter (YOLO 가 돔) " _
sleep 6; run "5 +look 사람 없음 (YOLO)"
bash $R/run/stop_all.sh quiet; pkill -f "camera_ros/camera_node" 2>/dev/null
echo "끝 (전부 끔)"
