#!/bin/bash
# Raspberry Pi 5 (Ubuntu Server 24.04, arm64) 를 로봇 컴퓨터로 만드는 설치 스크립트. [10-27 arm64 컨테이너, 10-28 실물 Pi 5 에서 전 과정 통과]
#
# 사용 (Pi에 SSH로 들어가서):
#   git clone --recurse-submodules https://github.com/yoosunu/semantic-companion-robot.git ~/semantic-companion-robot
#   bash ~/semantic-companion-robot/scripts/setup_pi.sh
# 다시 실행해도 안전하다 (이미 된 단계는 건너뜀). 30~60분 걸림 (대부분 apt 다운로드 + 빌드).
#
# 하는 일: ① 시스템 업데이트·기본 도구  ② ROS 2 Jazzy (화면 없는 ros-base) + Nav2·SLAM Toolbox·라이다 드라이버
#          ③ 시리얼 권한(dialout)  ④ ~/scr_ws 작업 공간 빌드 (Gazebo 패키지 제외)  ⑤ .bashrc 에 ROS 환경
# 하지 않는 일 (도착 날 실측 후 결정 → docs/ugv02-protocol.md day-1 체크리스트):
#   - UGV02 ↔ Pi 연결 포트(GPIO UART /dev/ttyAMA0 인지 USB 인지)와 UART 활성화 설정
#   - Wi-Fi·비밀번호 등 비밀값은 다루지 않음 (Imager 에서 이미 설정, repo 에 절대 넣지 않음)
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WS=~/scr_ws
step() { echo; echo "=== $* ==="; }

# 0) 환경 확인
. /etc/os-release
[ "${VERSION_CODENAME:-}" = "noble" ] || { echo "Ubuntu 24.04 (noble) 가 아님: ${PRETTY_NAME:-?}"; exit 1; }
echo "OS: $PRETTY_NAME / arch: $(uname -m)"
[ "$(uname -m)" = "aarch64" ] || echo "⚠ arm64 가 아님 (VM 컨테이너 시험이면 정상)"

# 시계 확인: 시계가 틀리면 apt 가 저장소를 "아직 유효하지 않음(not valid yet)"으로 거절한다.
# Pi 5 는 시계 배터리가 없어 부팅 직후 인터넷 시간을 받기 전까지 틀릴 수 있음 (10-27 VM에서 19시간 늦어 실제로 겪음)
if timedatectl show -p NTPSynchronized --value >/dev/null 2>&1; then
  for _ in $(seq 1 30); do [ "$(timedatectl show -p NTPSynchronized --value)" = "yes" ] && break; sleep 2; done
  [ "$(timedatectl show -p NTPSynchronized --value)" = "yes" ] || echo "⚠ 인터넷 시간 동기화가 아직 안 됨"
fi
echo "현재 시각: $(date '+%F %T %Z')  ← 실제 시각과 다르면 Ctrl+C 후: sudo systemctl restart systemd-timesyncd"
[ "$(date +%Y)" -ge 2026 ] || { echo "시계가 과거(年 $(date +%Y))로 되어 있음 → 시간부터 맞추기"; exit 1; }

step "1) 시스템 업데이트 + 기본 도구"
# Ubuntu Server 이미지에 noble-updates 가 빠져 있는 경우가 있음 (10-28 실제로 겪음 → docs/debugging/002)
# 없으면 설치된 라이브러리(updates 버전)와 -dev 패키지(release 버전)가 안 맞아 3) 에서 "unmet dependencies"
SRC=/etc/apt/sources.list.d/ubuntu.sources
if [ -f "$SRC" ] && ! grep -q "^Suites:.*${VERSION_CODENAME}-updates" "$SRC"; then
  sudo sed -i "s/^Suites: ${VERSION_CODENAME}\$/Suites: ${VERSION_CODENAME} ${VERSION_CODENAME}-updates ${VERSION_CODENAME}-backports/" "$SRC"
  echo "apt 소스에 ${VERSION_CODENAME}-updates/-backports 추가"
fi
sudo apt-get update
sudo DEBIAN_FRONTEND=noninteractive apt-get -y upgrade
sudo DEBIAN_FRONTEND=noninteractive apt-get install -y \
  locales curl git tmux htop socat software-properties-common \
  avahi-daemon \
  build-essential python3-pip python3-serial nlohmann-json3-dev
# avahi-daemon: Mac 에서 ssh jasper@scr-pi.local 로 찾을 수 있게 (Ubuntu Server 기본엔 없음)
sudo locale-gen en_US en_US.UTF-8 >/dev/null
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

step "2) ROS 2 Jazzy 저장소 등록"
if ! dpkg -s ros2-apt-source >/dev/null 2>&1; then
  sudo add-apt-repository -y universe
  # 공식 문서 방식 (docs.ros.org Jazzy 설치): ros2-apt-source 패키지가 키와 저장소를 등록
  VER=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F '"tag_name"' | awk -F'"' '{print $4}')
  [ -n "$VER" ] || { echo "ros-apt-source 버전을 못 받음 (인터넷/GitHub 확인)"; exit 1; }
  curl -fsSL -o /tmp/ros2-apt-source.deb \
    "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${VER}/ros2-apt-source_${VER}.${VERSION_CODENAME}_all.deb"
  sudo dpkg -i /tmp/ros2-apt-source.deb
  sudo apt-get update
else
  echo "이미 등록됨"
fi

step "3) ROS 2 Jazzy + 로봇에 필요한 패키지 (화면 없는 구성)"
sudo DEBIAN_FRONTEND=noninteractive apt-get install -y \
  ros-jazzy-ros-base ros-dev-tools \
  ros-jazzy-navigation2 ros-jazzy-nav2-bringup \
  ros-jazzy-slam-toolbox \
  ros-jazzy-robot-state-publisher ros-jazzy-xacro \
  ros-jazzy-tf2-tools ros-jazzy-robot-localization \
  ros-jazzy-teleop-twist-keyboard
# 라이다 드라이버는 apt rplidar_ros 가 아니라 src/sllidar_ros2 (submodule, 6) 에서 빌드) — C1 은 SDK 2.x 필요 (10-28)
# 없는 것: Gazebo, RViz (화면은 Mac/VM 쪽에서 띄움)

step "4) rosdep"
[ -f /etc/ros/rosdep/sources.list.d/20-default.list ] || sudo rosdep init
rosdep update

step "5) 시리얼 권한 (UGV02, 라이다)"
if id -nG "$USER" | grep -qw dialout; then echo "이미 dialout 그룹"; else
  sudo usermod -aG dialout "$USER"; echo "dialout 추가 → 다시 로그인해야 적용"; fi

step "6) 작업 공간 ~/scr_ws (src = repo/src 링크) 빌드"
git -C "$REPO" submodule update --init --recursive   # 항상 (새로 추가된 submodule 도 받게)
mkdir -p "$WS"
[ -e "$WS/src" ] || ln -s "$REPO/src" "$WS/src"
set +u; source /opt/ros/jazzy/setup.bash; set -u
rosdep install --from-paths "$WS/src" --ignore-src -r -y \
  --skip-keys "gazebo ros_gz ros_gz_bridge ros_gz_sim" || echo "⚠ 일부 의존성 설치 실패 → 위 메시지 확인"
cd "$WS"
# scr_gazebo(시뮬 전용)와 multirobot_map_merge(안 씀)는 Pi 에서 빌드하지 않음
colcon build --symlink-install --packages-skip scr_gazebo multirobot_map_merge

step "7) .bashrc 에 ROS 환경"
LINE_ROS='source /opt/ros/jazzy/setup.bash'
LINE_WS='[ -f ~/scr_ws/install/setup.bash ] && source ~/scr_ws/install/setup.bash'
grep -qxF "$LINE_ROS" ~/.bashrc || echo "$LINE_ROS" >> ~/.bashrc
grep -qxF "$LINE_WS" ~/.bashrc || echo "$LINE_WS" >> ~/.bashrc

step "완료"
echo "다음: 로그아웃 후 다시 SSH 접속 (dialout 적용) → ros2 pkg list | grep scr_"
echo "도착 날 체크리스트: docs/ugv02-protocol.md (UART 포트, 피드백 형식, 라이다 /scan)"
