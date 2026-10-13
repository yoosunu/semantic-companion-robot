#!/bin/bash
# simple_mover에 명령 보내기.  예) ./scripts/move.sh forward 1.0 | left 90 | right 45 | backward 0.5 | stop
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/env.sh"
[ $# -ge 1 ] || { echo "usage: move.sh <forward|backward|left|right|stop> [value]"; exit 1; }
ros2 topic pub --once /simple_mover/command std_msgs/msg/String "{data: '$*'}" > /dev/null && echo "sent: $*"
