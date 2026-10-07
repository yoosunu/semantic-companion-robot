#!/bin/bash
# 이 repo의 ROS 2 패키지를 빌드할 colcon 워크스페이스(~/scr_ws)를 만들고 빌드한다.
# 빌드 결과물(build/install/log)은 공유 폴더 밖(~/scr_ws)에 두고, src만 repo를 가리키는 심볼릭 링크로 연결.
# 재실행해도 안전하다. 사용: ./scripts/setup_workspace.sh
set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
WS=~/scr_ws

mkdir -p "$WS"
if [ ! -e "$WS/src" ]; then
  ln -s "$REPO_DIR/src" "$WS/src"
  echo "linked $WS/src -> $REPO_DIR/src"
fi

source /opt/ros/jazzy/setup.bash
[ -f ~/turtlebot3_ws/install/setup.bash ] && source ~/turtlebot3_ws/install/setup.bash
cd "$WS"
colcon build --symlink-install
echo
echo "built. packages:"; source "$WS/install/setup.bash"; ros2 pkg list | grep '^scr_' || true
