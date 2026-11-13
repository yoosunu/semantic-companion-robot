#!/bin/bash
# 공통 ROS 2 환경. 다른 스크립트에서 `source "$SCRIPT_DIR/env.sh"` 로 사용.
source /opt/ros/jazzy/setup.bash
[ -f ~/turtlebot3_ws/install/setup.bash ] && source ~/turtlebot3_ws/install/setup.bash
[ -f ~/scr_ws/install/setup.bash ]        && source ~/scr_ws/install/setup.bash
export TURTLEBOT3_MODEL=burger

# GUI(Gazebo/RViz)를 SSH 터미널(VS Code Remote-SSH)에서 실행해도 VM 화면에 뜨게 한다.
# Ubuntu 24.04 데스크톱은 Wayland + Xwayland(:0). SSH 세션에는 DISPLAY와 화면 접근 권한(XAUTHORITY)이 없으므로,
# VM에 로그인된 데스크톱 세션의 Xwayland 인증 파일(로그인할 때마다 이름이 바뀜)을 찾아 연결한다.
# 이미 DISPLAY가 있는 경우(VM 창의 터미널)는 건드리지 않는다.
if [ -z "$DISPLAY" ]; then
  _xauth="$(ls -t /run/user/$(id -u)/.mutter-Xwaylandauth.* 2>/dev/null | head -n1)"
  if [ -n "$_xauth" ]; then
    export DISPLAY=:0
    export XAUTHORITY="$_xauth"
  fi
  unset _xauth
fi
