#!/usr/bin/env python3
"""run_report.py — ROS 로그 폴더 → 한 주행의 시간표 (docs/design/logging.md 1층의 과도기 버전)

지금은 Nav2/브리지가 남긴 원본 로그(~/.ros/log/<노드>_<pid>_<시작ms>.log)를 읽어
사건(events)으로 바꾼다. 나중에 event_logger 가 events.jsonl 을 직접 쓰면 그걸 읽게 바꾼다.

  python3 scripts/run_report.py logs/living_1031              # 가장 최근 주행
  python3 scripts/run_report.py logs/living_1031 --list       # 폴더 안 주행 목록
  python3 scripts/run_report.py logs/living_1031 --run 2      # 목록의 2번 주행
  → logs/reports/<시작시각>.md, <시작시각>.events.jsonl

자동 (Pi, nav.launch.py 종료 시 · 부팅 시):
  run_report.py ~/.ros/log --pending --notify --prune 7 --out ~/scr_logs/reports
  --pending  보고서가 아직 없는 주행을 전부 (최근 7일)
  --notify   새 보고서 요약을 ntfy 로 한 번 (주제는 ~/.scr.env 의 NTFY_TOPIC, 저장소 밖)
  --prune N  N일 지난 ROS 로그(*.log)와 보고서 삭제 (docs/design/logging.md: 보관 7일)
"""
import argparse, json, math, os, re, sys, time, urllib.request
from collections import defaultdict
from pathlib import Path

LINE = re.compile(r"^\[(DEBUG|INFO|WARN|ERROR|FATAL)\] \[(\d+\.\d+)\] \[([^\]]+)\]: ?(.*)")
FNAME = re.compile(r"^(.+)_(\d+)_(\d{13})\.log$")
NUM = r"(-?\d+\.?\d*)"

STOP_ERROR_SEC = 30.0   # 충돌 정지가 이보다 길면 ERROR (logging.md)
STUCK_SEC = 45.0        # 같은 자리 이보다 오래 → 의심 구간
STUCK_RADIUS = 0.5      # m


# ---------------------------------------------------------------- 파일 → 주행 묶기
def scan_files(folder):
    """파일 이름의 시작 시각으로 주행(run)을 묶는다. bt_navigator 시작 = 주행 하나."""
    files = []
    for p in Path(folder).glob("*.log"):
        m = FNAME.match(p.name)
        if m:
            files.append((int(m.group(3)) / 1000.0, m.group(1), p))
    files.sort()
    anchors = [t for t, node, _ in files if node == "bt_navigator"]
    runs = []
    for i, a in enumerate(anchors):
        nxt = anchors[i + 1] if i + 1 < len(anchors) else float("inf")
        # 브리지·라이다는 Nav2 보다 몇 초 먼저, explore 는 몇 분 뒤에 뜰 수 있음
        members = [(t, n, p) for t, n, p in files
                   if a - 30 <= t < nxt - 30 and not n.startswith("map_saver")]
        runs.append((a, members))
    return runs


def read_lines(files):
    out = []
    for _, node, p in files:
        with open(p, encoding="utf-8", errors="replace") as f:
            for raw in f:
                m = LINE.match(raw.strip())
                if m:
                    out.append((float(m.group(2)), m.group(1), m.group(3), m.group(4), p.name))
    out.sort(key=lambda r: r[0])
    return out


# ---------------------------------------------------------------- 로그 줄 → 사건
class Builder:
    def __init__(self):
        self.ev = []
        self.pos = None            # 마지막으로 알려진 (x, y)
        self.pos_track = []        # (t, x, y)
        self.coll_state = "clear"
        self.coll_since = None
        self.plan_fail = None      # 묶는 중인 경로 실패 {goal, n, t0, t1, reason}
        self.extrap = None         # explore TF 에러 묶음
        self.pending_reason = {}   # behavior 실패 이유
        self.coll_counts = defaultdict(int)  # 감속·접근 횟수
        self.sense_counts = defaultdict(int)  # v7 IMU 감각 사건 수

    def add(self, t, lvl, src, ev, msg="", **kw):
        e = {"t": round(t, 2), "lvl": lvl, "src": src, "ev": ev}
        if self.pos:
            e["x"], e["y"] = round(self.pos[0], 2), round(self.pos[1], 2)
        if msg:
            e["msg"] = msg
        e.update(kw)
        self.ev.append(e)

    def set_pos(self, t, x, y):
        self.pos = (x, y)
        self.pos_track.append((t, x, y))

    def flush_plan_fail(self):
        p = self.plan_fail
        if p:
            self.ev.append({"t": round(p["t0"], 2), "lvl": "WARN", "src": "nav", "ev": "plan_failed",
                            "x": p["x"], "y": p["y"],
                            "msg": f"→ ({p['goal'][0]:.2f}, {p['goal'][1]:.2f}) {p['n']}회, "
                                   f"{p['t1'] - p['t0']:.0f}초간: {p['reason']}", "count": p["n"]})
            self.plan_fail = None

    def flush_extrap(self):
        x = self.extrap
        if x:
            self.ev.append({"t": round(x["t0"], 2), "lvl": "ERROR", "src": "explore", "ev": "pose_lost",
                            "msg": f"로봇 위치(TF)가 {x['last']:.1f} 에서 멈춤 — {x['n']}회, "
                                   f"{x['t1'] - x['t0']:.0f}초간", "count": x["n"]})
            self.extrap = None

    def line(self, t, lvl, node, msg):
        # --- 위치
        m = re.search(r"current location \(" + NUM + ", " + NUM + r"\) to \(" + NUM + ", " + NUM + r"\)", msg)
        if node == "bt_navigator" and m:
            self.set_pos(t, float(m.group(1)), float(m.group(2)))
            self.add(t, "INFO", "nav", "goal_sent", f"→ ({float(m.group(3)):.2f}, {float(m.group(4)):.2f})")
            return
        m = re.search(r"failed to plan from \(" + NUM + ", " + NUM + r"\) to \(" + NUM + ", " + NUM + r"\): \"?(.*?)\"?$", msg)
        if node == "planner_server" and m:
            x, y, gx, gy = map(float, m.group(1, 2, 3, 4))
            self.set_pos(t, x, y)
            reason = m.group(5)
            p = self.plan_fail
            if p and math.hypot(gx - p["goal"][0], gy - p["goal"][1]) < 0.5 and t - p["t1"] < 60:
                p["n"] += 1; p["t1"] = t
            else:
                self.flush_plan_fail()
                self.plan_fail = {"goal": (gx, gy), "n": 1, "t0": t, "t1": t,
                                  "reason": reason, "x": round(x, 2), "y": round(y, 2)}
            return
        if node == "planner_server" and "legal potential" in msg:
            return  # 바로 뒤 'failed to plan' 줄로 묶임

        # --- 목표 결과
        if node == "bt_navigator":
            if msg.startswith("Goal failed"):
                self.add(t, "WARN", "nav", "goal_failed")
            elif msg.startswith("Goal succeeded"):
                self.add(t, "INFO", "nav", "goal_ok")
            elif msg.startswith("Goal canceled"):
                self.add(t, "INFO", "nav", "goal_canceled")
            return
        if node == "controller_server" and "Failed to make progress" in msg:
            self.add(t, "WARN", "nav", "no_progress", "controller: Failed to make progress")
            return

        # --- 충돌 감시 (상태가 바뀔 때만)
        if node == "collision_monitor":
            new = None
            if "stop due to" in msg: new = "stop"
            elif "slowdown" in msg: new = "slow"
            elif "approach" in msg: new = "approach"
            elif "continue normal" in msg: new = "clear"
            if new and new != self.coll_state:
                was = self.coll_state
                self.end_collision(t)
                self.coll_state, self.coll_since = new, t
                # 감속·접근은 잦아서 시간표엔 안 쓰고 횟수만 (요약에 표시). 정지와 그 해제만 기록.
                if new == "stop":
                    self.add(t, "WARN", "collision", "stop", msg.split(" due to ")[-1])
                elif was == "stop":
                    self.add(t, "INFO", "collision", "stop_cleared", f"→ {new}")
                else:
                    self.coll_counts[new] += 1
            return

        # --- 복구 동작
        if node == "behavior_server":
            m = re.match(r"Running (\w+)$", msg)
            if m:
                self.add(t, "INFO", "recovery", "start", m.group(1)); return
            if "Exceeded time allowance" in msg or "Collision Ahead" in msg:
                self.pending_reason["last"] = msg.split(" - ")[0]; return
            m = re.match(r"(\w+) (completed successfully|failed)", msg)
            if m:
                ok = m.group(2).startswith("completed")
                self.add(t, "INFO" if ok else "WARN", "recovery", "ok" if ok else "failed",
                         m.group(1) + ("" if ok else f" ({self.pending_reason.pop('last', '?')})"))
            return

        # --- 탐색
        if node == "explore_node":
            if "All frontiers traversed" in msg:
                self.add(t, "INFO", "explore", "done", "모든 frontier 시도 완료")
            elif "Returning to initial pose" in msg:
                self.add(t, "INFO", "explore", "return_home")
            elif "Connected to move_base" in msg:
                self.add(t, "INFO", "run", "start", "explore 시작")
            elif "Extrapolation Error" in msg:
                m = re.search(r"latest data is at time " + NUM, msg)
                last = float(m.group(1)) if m else 0
                if self.extrap and t - self.extrap["t1"] < 30:
                    self.extrap["n"] += 1; self.extrap["t1"] = t
                else:
                    self.flush_extrap()
                    self.extrap = {"t0": t, "t1": t, "n": 1, "last": last}
            return

        # --- 브리지
        if node == "scr_explore" and msg.startswith("EXPLORE "):   # 11-06 우리 탐색기
            m = msg[8:]
            if m.startswith("시작"):
                self.add(t, "INFO", "run", "start", "scr_explore 시작")
            elif m.startswith("포기"):
                self.add(t, "WARN", "explore", "gave_up", m)
            elif m.startswith("남은 경계 없음"):
                self.add(t, "INFO", "explore", "return_home", m)
            elif m.startswith("끝"):
                self.add(t, "INFO" if "NOT" not in m else "WARN", "explore", "done", m)
            elif "STUCK" in m or "제자리" in m:
                self.add(t, "ERROR", "explore", "stuck", m)
            return

        if node == "whisker":   # 11-06 Whisker: 촉각 사건 + 상태 (빼기·갇힘·구조 요청)
            if msg.startswith("SENSE "):
                try:
                    d = json.loads(msg[6:])
                except ValueError:
                    return
                kind = d.get("ev", "?")
                self.sense_counts[kind] += 1
                if kind != "touch":
                    self.add(t, "INFO" if kind == "ghost" else "WARN", "whisker", kind,
                             f"{d.get('dir', '?')} {d.get('g', d.get('dps', ''))} @({d.get('x')}, {d.get('y')})"
                             + (" 표시" if d.get("marked") else ""))
            elif msg.startswith("WHISKER "):
                st = msg.split()[1].rstrip(":")
                if st in ("trapped", "help"):
                    self.add(t, "ERROR" if st == "help" else "WARN", "whisker", st, msg[8:])
                elif st == "drive":
                    self.sense_counts["escaped"] += 1
            return

        if node == "ugv02_bridge":
            if "odometry starts" in msg:
                self.add(t, "INFO", "bridge", "start", msg)
            elif msg.startswith("SENSE "):   # (v7 브리지 시절 형식, 11-06 v8 부터는 whisker 노드가 냄)
                try:
                    d = json.loads(msg[6:])
                except ValueError:
                    return
                kind = d.pop("ev", "?")
                self.sense_counts[kind] += 1
                self.add(t, "WARN" if kind in ("bump", "stall") else "INFO", "sense", kind,
                         " ".join(f"{k} {v}" for k, v in d.items()))
            return

    def end_collision(self, t):
        if self.coll_state == "stop" and self.coll_since is not None:
            dur = t - self.coll_since
            if dur >= STOP_ERROR_SEC:
                self.ev.append({"t": round(self.coll_since, 2), "lvl": "ERROR", "src": "collision",
                                "ev": "stop_long", "msg": f"충돌 정지가 {dur:.0f}초 지속 (해제 없이 끝났으면 종료 시각까지)"})

    def finish(self, t_end):
        self.flush_plan_fail()
        self.flush_extrap()
        self.end_collision(t_end)
        self.ev.sort(key=lambda e: e["t"])


# ---------------------------------------------------------------- 브리지 분석
def bridge_check(lines, t_end):
    """브리지가 종료 신호 없이 먼저 끊겼는지, 마지막 명령이 0이 아니었는지."""
    # 브리지 파일의 모든 줄 (rclcpp 의 "signal_handler" 줄은 로거 이름이 [rclcpp] 라서 노드 이름으로 거르면 놓침 — 11-04 오탐 3건)
    b = [r for r in lines if r[4].startswith("ugv02_bridge_")]
    if not b:
        return []
    out = []
    last_t = b[-1][0]
    shutdown = any("signal_handler" in r[3] for r in b)   # 정상 종료(Ctrl+C) 표시. 없으면 크래시·행·SIGHUP(tmux kill)
    last_cmd = None
    for r in b:
        m = re.search(r'"(?:X|L)":' + NUM + r',"(?:Z|R)":' + NUM, r[3])   # T:13 (X,Z) ~10-31, T:1/T:11 (L,R) 11-03~
        if m:
            last_cmd = (float(m.group(1)), float(m.group(2)))
    if not shutdown:
        msg = f"브리지가 정상 종료 표시(signal_handler) 없이 끝남 — 마지막 줄은 종료 {t_end - last_t:.0f}초 전"
        if last_cmd and (abs(last_cmd[0]) > 1e-3 or abs(last_cmd[1]) > 1e-3):
            msg += f" — 마지막 명령 {last_cmd[0]}, {last_cmd[1]} (0 아님)"
        out.append({"t": round(last_t, 2), "lvl": "ERROR", "src": "bridge", "ev": "died", "msg": msg})
    return out


def moving_summary(lines, t0, t_end, step=10):
    """브리지 'sent' 줄로 10초 구간별 움직임 명령 여부. 줄이 없으면 '-' (명령 없음/0 유지)."""
    bins = defaultdict(lambda: [0, 0])
    for t, _, node, msg, _ in lines:
        if node != "ugv02_bridge":
            continue
        m = re.search(r'"(?:X|L)":' + NUM + r',"(?:Z|R)":' + NUM, msg)
        if m:
            b = int((t - t0) // step)
            bins[b][1] += 1
            if abs(float(m.group(1))) > 1e-3 or abs(float(m.group(2))) > 1e-3:
                bins[b][0] += 1
    n = int((t_end - t0) // step) + 1
    return "".join("#" if bins[i][0] else ("." if bins[i][1] else " ") for i in range(n))


def stuck_segments(track):
    """같은 자리(반경 STUCK_RADIUS)에 STUCK_SEC 이상 머문 구간."""
    out, i = [], 0
    while i < len(track):
        t0, x0, y0 = track[i]
        j = i
        while j + 1 < len(track) and math.hypot(track[j + 1][1] - x0, track[j + 1][2] - y0) < STUCK_RADIUS:
            j += 1
        if track[j][0] - t0 >= STUCK_SEC:
            out.append((t0, track[j][0], x0, y0))
        i = j + 1
    return out


# ---------------------------------------------------------------- 출력
def fmt_t(t, t0):
    s = int(round(t - t0))
    return f"{s // 60:d}:{s % 60:02d}"


def report(run_start, files, out_dir):
    lines = read_lines(files)
    if not lines:
        sys.exit("로그 줄이 없음")
    t_end = max(r[0] for r in lines)
    # 주행 끝 = Nav2(bt_navigator) 가 종료 신호를 받은 때. explore 를 먼저 꺼도 Nav2 가 도는 동안은 주행으로 본다.
    sig = [r[0] for r in lines if "signal_handler" in r[3] and r[2] == "bt_navigator"] or \
          [r[0] for r in lines if "signal_handler" in r[3]]
    if sig:
        t_end = min(sig)

    b = Builder()
    for t, lvl, node, msg, _ in lines:
        if t <= t_end + 1:
            b.line(t, lvl, node, msg)
    b.finish(t_end)
    ev = b.ev + bridge_check(lines, t_end)
    ev.sort(key=lambda e: e["t"])
    starts = [e["t"] for e in ev if e["src"] == "run" and e["ev"] == "start"] or [lines[0][0]]
    t0 = starts[0]
    ev.append({"t": round(t_end, 2), "lvl": "INFO", "src": "run", "ev": "end", "msg": "종료 신호"})

    name = time.strftime("%Y%m%d_%H%M", time.localtime(run_start))
    out_dir = Path(out_dir); out_dir.mkdir(parents=True, exist_ok=True)
    with open(out_dir / f"{name}.events.jsonl", "w") as f:
        for e in ev:
            f.write(json.dumps(e, ensure_ascii=False) + "\n")

    cnt = defaultdict(int)
    for e in ev:
        cnt[e["lvl"]] += 1
    L = []
    L.append(f"# 주행 보고서 {time.strftime('%Y-%m-%d %H:%M', time.localtime(t0))}\n")
    L.append(f"- 길이: {fmt_t(t_end, t0)} (분:초) · 사건 {len(ev)}개 — "
             f"ERROR {cnt['ERROR']}, WARN {cnt['WARN']}, INFO {cnt['INFO']}")
    cc = b.coll_counts
    L.append(f"- 충돌 감시 감속 {cc['slow']}회, 접근 {cc['approach']}회 (시간표엔 생략)")
    sc = b.sense_counts
    if sc:
        L.append("- Whisker: " + ", ".join(f"{k} {v}회" for k, v in sorted(sc.items())))
    L.append(f"- 로그 파일 {len(files)}개: " + ", ".join(sorted({n for _, n, _ in files})))
    L.append("")
    L.append("## 움직임 (10초 한 칸 · `#` 움직이라는 명령 · `.` 0 명령 · 빈칸 명령 기록 없음)\n")
    L.append("```\n" + moving_summary(lines, t0, t_end) + "\n```\n")

    errs = [e for e in ev if e["lvl"] == "ERROR"]
    if errs:
        L.append("## ERROR\n")
        for e in errs:
            L.append(f"- **{fmt_t(e['t'], t0)}** `{e['src']}.{e['ev']}` {e.get('msg', '')}")
        L.append("")
    st = stuck_segments(b.pos_track)
    if st:
        L.append(f"## 의심 구간 (같은 자리 {STUCK_SEC:.0f}초+)\n")
        for a, z, x, y in st:
            near = [e for e in ev if a <= e["t"] <= z and e["lvl"] != "INFO"]
            kinds = defaultdict(int)
            for e in near:
                kinds[f"{e['src']}.{e['ev']}"] += e.get("count", 1)
            L.append(f"- {fmt_t(a, t0)}~{fmt_t(z, t0)} ({z - a:.0f}초) @ ({x:.2f}, {y:.2f}) — "
                     + (", ".join(f"{k}×{v}" for k, v in kinds.items()) or "WARN 없음"))
        L.append("")
    L.append("## 시간표\n")
    L.append("| 시각 | 수준 | 사건 | 위치 | 내용 |")
    L.append("|---|---|---|---|---|")
    for e in ev:
        pos = f"({e['x']:.2f}, {e['y']:.2f})" if "x" in e else ""
        lv = {"ERROR": "**ERROR**", "WARN": "WARN", "INFO": ""}[e["lvl"]]
        L.append(f"| {fmt_t(e['t'], t0)} | {lv} | {e['src']}.{e['ev']} | {pos} | {e.get('msg', '')} |")
    L.append("\n> 위치는 그 사건 직전에 로그에 찍힌 마지막 좌표 (map, m). 원본 로그는 Pi `~/.ros/log`.")
    md = out_dir / f"{name}.md"
    md.write_text("\n".join(L) + "\n")
    return md


def summarize(md_path):
    """보고서 첫 줄들에서 알림용 한 줄."""
    head = Path(md_path).read_text().splitlines()[2]  # "- 길이: ... · 사건 N개 — ERROR a, WARN b, INFO c"
    m = re.search(r"길이: (\S+).*ERROR (\d+), WARN (\d+)", head)
    return (m.group(1), int(m.group(2)), int(m.group(3))) if m else ("?", 0, 0)


def notify(lines, any_error):
    env = Path.home() / ".scr.env"
    topic = None
    if env.exists():
        for ln in env.read_text().splitlines():
            if ln.startswith("NTFY_TOPIC="):
                topic = ln.split("=", 1)[1].strip()
    if not topic:
        print("알림 생략: ~/.scr.env 에 NTFY_TOPIC 없음")
        return
    req = urllib.request.Request("https://ntfy.sh/" + topic, data="\n".join(lines).encode(), headers={
        "Title": "robot: run ended" + (" - ERROR" if any_error else ""),
        "Priority": "high" if any_error else "default",
        "Tags": "warning" if any_error else "robot"})
    try:
        urllib.request.urlopen(req, timeout=10)
    except Exception as e:  # 알림 실패가 보고서를 막으면 안 됨
        print("알림 실패:", e)


def prune(folder, out, days):
    cut = time.time() - days * 86400
    n = 0
    for p in list(Path(folder).glob("*.log")) + list(Path(out).glob("*.md")) + list(Path(out).glob("*.events.jsonl")):
        if FNAME.match(p.name) or p.parent == Path(out):
            if p.stat().st_mtime < cut:
                p.unlink(); n += 1
    if n:
        print(f"{days}일 지난 파일 {n}개 삭제")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("folder")
    ap.add_argument("--list", action="store_true")
    ap.add_argument("--run", type=int, help="--list 의 번호 (기본: 마지막)")
    ap.add_argument("--pending", action="store_true", help="보고서가 없는 주행 전부 (최근 7일)")
    ap.add_argument("--notify", action="store_true")
    ap.add_argument("--prune", type=int, metavar="DAYS")
    ap.add_argument("--out", default=str(Path(__file__).resolve().parent.parent / "logs" / "reports"))
    a = ap.parse_args()
    a.folder, a.out = os.path.expanduser(a.folder), os.path.expanduser(a.out)
    runs = scan_files(a.folder)
    if a.list:
        for i, (t, fs) in enumerate(runs, 1):
            print(f"{i}: {time.strftime('%m-%d %H:%M:%S', time.localtime(t))}  파일 {len(fs)}개")
        return
    if a.pending:
        done = {p.name[:13] for p in Path(a.out).glob("*.md")} if Path(a.out).exists() else set()
        todo = [(t, fs) for t, fs in runs
                if time.strftime("%Y%m%d_%H%M", time.localtime(t)) not in done and t > time.time() - 7 * 86400]
    elif runs and (a.run or not a.prune):
        todo = [runs[(a.run or len(runs)) - 1]]
    else:
        todo = []
    made = []
    for t, fs in todo:
        md = report(t, fs, a.out)
        print(md)
        made.append(md)
    if a.notify and made:
        lines, err = [], False
        for md in made:
            dur, e, w = summarize(md)
            err |= e > 0
            lines.append(f"{Path(md).stem}  {dur}  ERROR {e} / WARN {w}")
        notify(lines, err)
    if a.prune:
        prune(a.folder, a.out, a.prune)
    if not runs and not a.prune:
        sys.exit("bt_navigator 로그가 없음 — Nav2 주행 로그 폴더가 맞는지 확인")


if __name__ == "__main__":
    main()
