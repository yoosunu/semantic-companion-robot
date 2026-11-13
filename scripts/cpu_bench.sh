#!/bin/bash
# [Pi] CPU 재기 (11-13): 모듈을 하나씩 더 켜면서 프로세스별 CPU·온도·Pi 전력. 주행 안 함. 약 3분.
#   bash ~/semantic-companion-robot/scripts/cpu_bench.sh [단계당 초=25]
#   재는 동안 카메라 앞에 앉아 있기 (얼굴이 보이는 상태 = "같이 있기" 모드 기준). 스트림은 안 켬 (JPEG 인코딩도 CPU 라서)
R=~/semantic-companion-robot; S=${1:-25}; P="python3 $R/scripts/cpu_probe.py $S"
PY="source ~/scr_ws/install/setup.bash; ~/yolo_venv/bin/python -u $R/scripts"
bash $R/scripts/stop_all.sh quiet; pkill -f "camera_ros/camera_node" 2>/dev/null; sleep 3
$P "0 바탕 (ROS 없음)"
bash $R/scripts/look_test.sh > /dev/null; sleep 5
$P "A 같이있기: 브리지+LiDAR+카메라640+얼굴+고개"
tmux new -d -s yolo "bash -ic '$PY/yolo_lab.py --model ~/models/yolo26n.onnx --hz 2 > /tmp/yolo.log 2>&1'"; sleep 8
$P "B A + YOLO26 2Hz"
tmux new -d -s floor "bash -ic '$PY/floor_lab.py --hz 5 > /tmp/floor.log 2>&1'"; sleep 5
$P "C B + 바닥 5Hz"
tmux kill-session -t yolo; sleep 1
tmux new -d -s yolo "bash -ic '$PY/yolo_lab.py --model ~/models/yoloe-26s-seg.onnx --conf 0.05 --hz 0.5 > /tmp/yolo.log 2>&1'"; sleep 10
$P "D A + 바닥 + YOLOE 2초에 1번 (YOLO26 대신)"
tmux kill-session -t yolo; sleep 1
tmux new -d -s yolo "bash -ic '$PY/yolo_lab.py --model ~/models/yolo26n.onnx > /tmp/yolo.log 2>&1'"; sleep 8
$P "E 최악: A + 바닥 + YOLO26 매 영상"
bash $R/scripts/stop_all.sh quiet; pkill -f "camera_ros/camera_node" 2>/dev/null
echo "끝 (전부 끔). 모델 로그: /tmp/yolo.log /tmp/floor.log"
