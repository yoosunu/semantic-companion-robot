#!/bin/bash
# [Pi] 팬틸트 카메라(OV5647) 를 Ubuntu 24.04 + Pi 5 + ROS 2 Jazzy 에서 쓰기 위한 설치 (한 번만).
#   bash ~/semantic-companion-robot/scripts/setup_camera.sh
#
# 왜 소스 빌드가 꼭 필요한가 (11-12 확인):
#   - Pi 5 카메라는 새 영상 처리기(PiSP)를 거침. Ubuntu 공식 문서: 카메라 스택은 "Ubuntu 25.04 부터" 지원 → 24.04 기본 libcamera 는 안 됨
#   - ROS apt 의 libcamera / camera_ros 도 원본(upstream) libcamera 라 PiSP 없음 → Raspberry Pi 판 libcamera 를 빌드하고
#     camera_ros 를 그 위에 다시 빌드해야 함 (camera_ros 저자 안내와 같음)
#   - rpicam-apps 는 안 함 (사진 시험용일 뿐, ROS 에 필요 없음). 확인은 libcamera 에 들어 있는 `cam -l` 로
# 결과물: /usr/local 의 libcamera (+ cam 도구), ~/cam_ws 의 camera_ros. 저장소 안에는 아무것도 안 만듦.
set -e
step() { echo; echo "=== $* ==="; }

step "1) 빌드 도구·라이브러리"
sudo apt update
sudo apt install -y git meson ninja-build pkg-config cmake python3-yaml python3-ply python3-jinja2 \
  libyaml-dev libgnutls28-dev openssl libdw-dev libunwind-dev libudev-dev libevent-dev libdrm-dev \
  libjpeg-dev libtiff-dev libboost-dev libboost-program-options-dev

step "2) ROS 판 libcamera 가 깔려 있으면 지움 (PiSP 없음 — 실행 때 이게 먼저 잡히면 카메라 안 보임)"
for p in ros-jazzy-camera-ros ros-jazzy-libcamera; do
  if dpkg -s "$p" >/dev/null 2>&1; then sudo apt remove -y "$p"; else echo "$p 없음 (좋음)"; fi
done

step "3) Raspberry Pi 판 libcamera 빌드 (Pi 5 에서 10~20분)"
mkdir -p ~/src && cd ~/src
[ -d libcamera ] || git clone --depth 1 https://github.com/raspberrypi/libcamera.git
cd libcamera
[ -d build ] || meson setup build --buildtype=release \
  -Dpipelines=rpi/vc4,rpi/pisp -Dipas=rpi/vc4,rpi/pisp -Dv4l2=true -Dgstreamer=disabled \
  -Dtest=false -Dlc-compliance=disabled -Dcam=enabled -Dqcam=disabled -Ddocumentation=disabled -Dpycamera=disabled
ninja -C build -j 3
sudo ninja -C build install
sudo ldconfig

step "4) 카메라 보이나 (ov5647 이 나와야 함)"
sudo usermod -aG video "$USER"
cam -l || true

step "5) camera_ros 를 위 libcamera 로 빌드 (~/cam_ws)"
source /opt/ros/jazzy/setup.bash
mkdir -p ~/cam_ws/src && cd ~/cam_ws/src
[ -d camera_ros ] || git clone --depth 1 https://github.com/christianrauch/camera_ros.git
cd ~/cam_ws
rosdep install -y --from-paths src --ignore-src --skip-keys=libcamera
export PKG_CONFIG_PATH=/usr/local/lib/aarch64-linux-gnu/pkgconfig:/usr/local/lib/pkgconfig:$PKG_CONFIG_PATH
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

step "6) .bashrc 에 cam_ws 추가 (scr_ws 다음)"
LINE='[ -f ~/cam_ws/install/setup.bash ] && source ~/cam_ws/install/setup.bash'
grep -qxF "$LINE" ~/.bashrc || echo "$LINE" >> ~/.bashrc

echo
echo "끝. 새 터미널을 열고(그룹·환경 적용):  cam -l   →  ov5647 이 보이면 성공"
echo "ROS 시험: bash ~/semantic-companion-robot/scripts/cam_test.sh"
