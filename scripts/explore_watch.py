#!/usr/bin/env python3
"""[Pi] 탐색기(explore_lite)를 켜고 지켜보다가, 끝나면 지도를 저장하고 폰으로 알린다.

  python3 scripts/explore_watch.py                 # tmux 창 안에서 (아래 예)
  python3 scripts/explore_watch.py --name room     # 지도 이름 앞부분 (기본 explore)

  tmux new -d -s explore "bash -ic 'python3 ~/semantic-companion-robot/scripts/explore_watch.py; exec bash'"

왜 (11-06 Jasper): "명령을 보내고 끝나버리니 고민 중인지 끝났는지 모른다." → 탐색기 로그를 읽어서
  - "Exploration stopped"            : 더 볼 곳 없음 → 집으로 복귀 시작 (알림 안 함, 화면에만)
  - "returned to initial pose"       : 복귀까지 끝 → 지도 자동 저장 ~/maps/<name>_<날짜_시각> + ntfy 알림
  - 복귀가 3분 안에 안 끝나면         : 저장 + "복귀 못 함" 알림 (막혔을 수 있음)
  - 탐색 중 3분 동안 10 cm 도 못 움직이면: "멈춘 것 같음" 알림 (끝나지도 않고 서 버린 경우. 다시 움직이면 다시 감시) — 11-06
알림 주제는 ~/.scr.env 의 NTFY_TOPIC (저장소 밖). 탐색기 자체는 그대로 둠 → 끄는 건 평소처럼 Ctrl+C.
"""
import argparse
import math
import os
import subprocess
import threading
import time
import urllib.request
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
RETURN_LIMIT = 180.0   # s, "Exploration stopped" 뒤 복귀를 기다리는 최대 시간
STUCK_LIMIT = 180.0    # s, 탐색 중 이 시간 동안 STUCK_DIST 도 못 움직이면 알림
STUCK_DIST = 0.10      # m


def odom_watch(state):
    """/odom 을 듣는 작은 노드 (별도 스레드). 마지막으로 10 cm 이상 움직인 시각을 state 에 적는다."""
    import rclpy
    from nav_msgs.msg import Odometry
    rclpy.init()
    n = rclpy.create_node("explore_watch")
    anchor = {"p": None}

    def on_odom(m):
        p = (m.pose.pose.position.x, m.pose.pose.position.y)
        if anchor["p"] is None or math.dist(p, anchor["p"]) > STUCK_DIST:
            anchor["p"] = p
            state["moved_at"] = time.time()
            state["stuck_sent"] = False
    n.create_subscription(Odometry, "odom", on_odom, 10)
    try:
        rclpy.spin(n)
    except Exception:
        pass


def ntfy(title, body, urgent=False):
    topic = None
    env = Path.home() / ".scr.env"
    if env.exists():
        for ln in env.read_text().splitlines():
            if ln.startswith("NTFY_TOPIC="):
                topic = ln.split("=", 1)[1].strip()
    if not topic:
        print("[watch] 알림 생략: ~/.scr.env 에 NTFY_TOPIC 없음", flush=True)
        return
    req = urllib.request.Request("https://ntfy.sh/" + topic, data=body.encode(), headers={
        "Title": title, "Priority": "high" if urgent else "default", "Tags": "warning" if urgent else "robot"})
    try:
        urllib.request.urlopen(req, timeout=10)
    except Exception as e:   # 알림이 실패해도 탐색은 계속
        print("[watch] 알림 실패:", e, flush=True)


def save_map(name):
    path = Path.home() / "maps" / f"{name}_{time.strftime('%Y%m%d_%H%M')}"
    path.parent.mkdir(exist_ok=True)
    r = subprocess.run(["ros2", "run", "nav2_map_server", "map_saver_cli", "-f", str(path),
                        "--ros-args", "-p", "save_map_timeout:=10.0"], capture_output=True, text=True)
    ok = "Map saved successfully" in (r.stdout + r.stderr)
    print(f"[watch] 지도 저장 {'OK' if ok else '실패'}: {path}", flush=True)
    return path, ok


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--name", default="explore", help="지도 이름 앞부분 (기본 explore)")
    a = ap.parse_args()

    env = dict(os.environ, RCUTILS_LOGGING_BUFFERED_STREAM="0")   # 로그 줄이 바로바로 오게
    cmd = ["ros2", "run", "explore_lite", "explore", "--ros-args",
           "--params-file", str(REPO / "config" / "explore.yaml"), "-p", "use_sim_time:=false"]
    p = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, env=env)
    t0 = time.time()
    state = {"stopped_at": None, "done": False, "moved_at": time.time(), "stuck_sent": False}   # (목표 개수는 탐색기가 DEBUG 로만 찍어서 못 셈)

    def finish(title, urgent):
        if state["done"]:
            return
        state["done"] = True
        path, ok = save_map(a.name)
        mins, secs = divmod(int(time.time() - t0), 60)
        body = f"{mins}분 {secs}초\n지도: {path.name}{'' if ok else ' (저장 실패!)'}"
        print(f"\n[watch] ===== {title} — {body.replace(chr(10), ' / ')} =====\n", flush=True)
        ntfy(title, body, urgent or not ok)

    def return_timer():   # 복귀가 너무 오래 걸리면 그래도 알린다 + 탐색 중 멈춤 감시
        while not state["done"]:
            if state["stopped_at"] and time.time() - state["stopped_at"] > RETURN_LIMIT:
                finish("robot: explore done, NOT home", True)
            if (not state["stopped_at"] and not state["stuck_sent"]
                    and time.time() - state["moved_at"] > STUCK_LIMIT):
                state["stuck_sent"] = True
                mins = int((time.time() - t0) // 60)
                print(f"\n[watch] ===== {STUCK_LIMIT / 60:.0f}분 동안 제자리 — 멈춘 것 같음 ({mins}분째) =====\n", flush=True)
                ntfy("robot: explore STUCK?", f"{STUCK_LIMIT / 60:.0f}분 동안 10 cm 도 못 움직임 (탐색 {mins}분째). RViz 로 확인", True)
            time.sleep(2)
    threading.Thread(target=odom_watch, args=(state,), daemon=True).start()
    threading.Thread(target=return_timer, daemon=True).start()

    try:
        for line in p.stdout:
            print(line, end="", flush=True)
            low = line.lower()
            if "exploration stopped" in low and not state["stopped_at"]:
                state["stopped_at"] = time.time()
                print("[watch] 더 볼 곳 없음 → 집으로 복귀 중…", flush=True)
            if "returned to initial pose" in low:
                finish("robot: explore done", False)
            if "failed to return" in low:
                finish("robot: explore done, NOT home", True)
    except KeyboardInterrupt:
        pass
    finally:
        if p.poll() is None:
            p.terminate()
            p.wait(timeout=10)


if __name__ == "__main__":
    main()
