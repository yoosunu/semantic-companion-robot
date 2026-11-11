#!/bin/bash
# [Pi] YOLO(사람 찾기)용 onnxruntime 설치 (한 번만). 시스템 파이썬(ROS rclpy)을 그대로 쓰는 가상환경 ~/yolo_venv.
#   bash ~/semantic-companion-robot/scripts/setup_yolo.sh
# 모델 파일은 Mac 에서: scp models/yolo11n.onnx jasper@<Pi>:models/   (11-12: YOLO11n, 입력 320, onnx 10 MB)
# 왜 ultralytics(pytorch) 를 안 깔았나: Pi 에선 무겁고 느림. 모델은 미리 onnx 로 바꿔 두고 onnxruntime 으로만 돌림
set -e
mkdir -p ~/models
# 11-12: python3-venv 가 없으면 venv 가 pip 없이 반쯤 만들어짐 → 패키지 설치 후 다시 만듦
dpkg -s python3-venv >/dev/null 2>&1 || sudo apt install -y python3-venv
[ -x ~/yolo_venv/bin/pip ] || { rm -rf ~/yolo_venv; python3 -m venv --system-site-packages ~/yolo_venv; }
~/yolo_venv/bin/pip install -q onnxruntime numpy
~/yolo_venv/bin/python -c "import onnxruntime, numpy; print('onnxruntime', onnxruntime.__version__, 'numpy', numpy.__version__)"
~/yolo_venv/bin/python -c "import cv2; print('cv2', cv2.__version__)" 2>/dev/null || echo "cv2 없음 → numpy 로 줄임 (조금 느림, 괜찮음)"
ls -la ~/models/yolo11n.onnx 2>/dev/null || echo "⚠ ~/models/yolo11n.onnx 없음 → Mac 에서 scp 먼저"
