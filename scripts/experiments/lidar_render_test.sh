#!/bin/bash
# gpu_lidar가 렌더러 조건별로 정상 거리값을 내는지 비교한다.
# TurtleBot과 무관한 Gazebo 공식 예제 월드(gpu_lidar_sensor.sdf) 사용, 서버만 실행(GUI 없음).
# 사용: robot-stop 으로 기존 시뮬레이션을 끈 뒤 ./scripts/experiments/lidar_render_test.sh

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOG_DIR="$SCRIPT_DIR/../../logs"
mkdir -p "$LOG_DIR"
OUT="$LOG_DIR/lidar_render_test_$(date +%Y%m%d_%H%M%S).txt"
WORLD=gpu_lidar_sensor.sdf
TOPIC=/lidar

stats() {
  awk '/^ranges:/ {
         n++; v=$2
         if (v=="inf" || v=="-inf") inf++
         else if (v=="nan" || v=="-nan") nan++
         else { f++; if (min=="" || v+0<min) min=v+0; if (v+0>max) max=v+0; seen[v]=1 }
       }
       END {
         d=0; for (k in seen) d++
         printf "beams=%d finite=%d inf=%d nan=%d min=%s max=%s distinct_finite_values=%d\n", n, f, inf, nan, min, max, d
       }'
}

run_case() {
  local name="$1"; shift
  echo "=================================================="
  echo "### CASE: $name"
  echo "cmd: $*"
  pkill -f "gz sim" 2>/dev/null; sleep 2

  "$@" > "$LOG_DIR/.gz_server.log" 2>&1 &
  local pid=$!
  sleep 10

  if ! kill -0 $pid 2>/dev/null; then
    wait $pid; echo "RESULT: server exited early (exit code $?)"
    echo "--- server log tail ---"; tail -n 15 "$LOG_DIR/.gz_server.log"
    return
  fi

  local msg
  msg="$(timeout 10 gz topic -e -t $TOPIC -n 1 2>&1)"
  if [ -z "$msg" ]; then
    echo "RESULT: no lidar message within 10s"
  else
    echo "RESULT: $(echo "$msg" | stats)"
    echo "first 8 ranges: $(echo "$msg" | awk '/^ranges:/{print $2}' | head -n 8 | tr '\n' ' ')"
  fi
  echo "--- server log tail ---"; tail -n 8 "$LOG_DIR/.gz_server.log"

  kill $pid 2>/dev/null; sleep 2; kill -9 $pid 2>/dev/null
}

{
  echo "=== $(date) ==="
  echo "### OpenGL info"
  (command -v glxinfo >/dev/null && glxinfo -B 2>&1 | grep -E "OpenGL (renderer|core profile version|version)") || echo "glxinfo not installed (sudo apt install mesa-utils)"
  echo "### gz version: $(gz sim --versions 2>/dev/null | head -n1)"

  run_case "1. ogre"                         gz sim -s -r -v 3 $WORLD --render-engine-server ogre
  run_case "2. ogre2"                        gz sim -s -r -v 3 $WORLD --render-engine-server ogre2
  run_case "3. ogre2 + software GL"          env LIBGL_ALWAYS_SOFTWARE=1 gz sim -s -r -v 3 $WORLD --render-engine-server ogre2
  run_case "4. ogre2 + headless rendering"   gz sim -s -r -v 3 $WORLD --render-engine-server ogre2 --headless-rendering

  pkill -f "gz sim" 2>/dev/null
  rm -f "$LOG_DIR/.gz_server.log"
  echo "=== done ==="
} > "$OUT" 2>&1

echo "saved: $OUT"
