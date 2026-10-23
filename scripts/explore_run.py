#!/usr/bin/env python3
"""스스로 탐색: 목표 좌표 없이 explore_lite(기성품 frontier 탐색기)가 갈 곳을 고른다. 순회(tour)와 같은 방식으로 채점.

  robot explore              (SLAM 모드에서만: robot up slam)
  Ctrl+C                     탐색 중단 → 지금까지로 채점

끝나는 조건: 탐색기가 "더 갈 경계 없음"을 알리고 출발점 복귀까지 마침 / 시간 제한(기본 15분) / Ctrl+C
채점: 경로 오차 ATE(정답 대비), 출발점 복귀 표류, 이동 거리, 시간, 지도 완성도(기준 지도 빈 공간 중 %)
기록: logs/explore_<시각>.txt / .csv, 지도 logs/maps/explore_<시각>.{pgm,yaml}
설정: config/explore.yaml (경계 점수 = potential_scale × 거리 − gain_scale × 크기)
"""
import argparse
import math
import os
import subprocess
import threading
import time
from datetime import datetime

import rclpy
from rclpy.signals import SignalHandlerOptions

from shuttle import Shuttle, wrap
from tour import REPO, Recorder, align, best_fit, coverage, distance, gazebo_truth, my_settings, save_map

DONE_WORDS = ("exploration stopped", "no frontiers", "all frontiers", "returning to initial pose")


def main():
    ap = argparse.ArgumentParser(description="스스로 탐색 + 채점")
    ap.add_argument("--mode", default="", help="robot 이 자동으로 붙임")
    ap.add_argument("--limit", type=float, default=15.0, help="시간 제한 [분]")
    args = ap.parse_args()
    if args.mode != "slam":
        print("탐색은 SLAM 모드에서만 → robot down → robot up slam"); return
    if gazebo_truth() is None:
        print("Gazebo 실제 위치를 못 읽음. 시뮬이 켜져 있나요? → robot status"); return
    if subprocess.run(["ros2", "pkg", "prefix", "explore_lite"], capture_output=True).returncode != 0:
        print("explore_lite 없음 → ~/scr_ws 에서 m-explore-ros2 빌드 후 source (docs/sim-playbook.md)"); return

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Shuttle()   # 위치 읽기(TF), Nav2 준비 확인만 사용. 목표는 탐색기가 보냄
    stamp = f"{datetime.now():%Y%m%d_%H%M%S}"
    rec = proc = None
    state = {"done": False, "returning": False, "goals": 0}

    def watch(stream):   # 탐색기 출력을 그대로 보여주면서 "끝" 신호를 찾는다
        for line in stream:
            line = line.rstrip()
            print(f"  [explore] {line}")
            low = line.lower()
            if "sending goal" in low or "new goal" in low:
                state["goals"] += 1
            if "returning to initial pose" in low:
                state["returning"] = True
            elif any(w in low for w in DONE_WORDS):
                state["done"] = True

    reason = "?"
    t_start = time.monotonic()
    try:
        print("준비 확인 중...")
        while not (node.nav.server_is_ready() and node.nav2_ready() and node.robot_pose() is not None):
            node.settle(1.0)
            print("  아직... 오래 걸리면 robot status")
        rec = Recorder(node); rec.start()
        t_start = time.monotonic()
        proc = subprocess.Popen(
            ["ros2", "run", "explore_lite", "explore", "--ros-args",
             "--params-file", os.path.join(REPO, "config", "explore.yaml"), "-p", "use_sim_time:=true"],
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1)
        threading.Thread(target=watch, args=(proc.stdout,), daemon=True).start()
        print(f"탐색 시작 (제한 {args.limit:.0f}분). RViz에서 경계 표시(/explore/frontiers)를 켜면 고르는 후보가 보여요\n")

        returned_at = None
        while True:
            node.settle(1.0)
            elapsed = time.monotonic() - t_start
            if proc.poll() is not None:
                reason = "탐색기 종료"; break
            if state["done"]:
                # 복귀 중이면 로봇이 멈출 때까지 조금 더 기다린다
                if returned_at is None:
                    returned_at = time.monotonic()
                if time.monotonic() - returned_at > (60 if state["returning"] else 5):
                    reason = "경계 없음 (탐색 완료)"; break
            if elapsed > args.limit * 60:
                reason = f"시간 제한 {args.limit:.0f}분"; break
    except KeyboardInterrupt:
        reason = "Ctrl+C"
    finally:
        total = time.monotonic() - t_start
        if proc and proc.poll() is None:
            proc.terminate()
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                proc.kill()
        node.cancel()
        if rec:
            node.settle(1.0); rec.running = False; rec.join(timeout=3)

    try:
        if not rec or len(rec.samples) < 2:
            print("기록이 부족해서 채점 못 함"); return
        rot_deg, errs = best_fit(rec.samples)
        rmse = math.sqrt(sum(e * e for e in errs) / len(errs))
        samples = align(rec.samples)
        _, tr_end, es_end = samples[-1]
        drift = math.hypot(tr_end[0] - es_end[0], tr_end[1] - es_end[1])
        yaw_end = math.degrees(abs(wrap(tr_end[2] - es_end[2])))
        home = math.hypot(tr_end[0] - samples[0][1][0], tr_end[1] - samples[0][1][1])

        summary = [
            f"=== explore {stamp}  (끝난 이유: {reason}) ===",
            f"총 {total:.1f} s, 샘플 {len(samples)}개",
            f"이동 거리 약 {distance(samples):.1f} m",
            f"경로 오차 ATE: RMSE {rmse * 100:.1f} cm, 최대 {max(errs) * 100:.1f} cm  (지도 전체 회전 {rot_deg:+.1f}°)",
            f"최종 표류 (출발 기준): 위치 {drift * 100:.1f} cm, 방향 {yaw_end:.1f}°  | 끝난 위치 ~ 출발점 거리 {home:.2f} m",
            "설정 explore.yaml: " + " | ".join(my_settings(os.path.join(REPO, "config", "explore.yaml"))[:6]) + " ...",
        ]
        print("\n지도 저장 중...")
        path = save_map(stamp, prefix="explore")
        if path:
            summary.append(f"지도 완성도: 기준 지도 빈 공간의 {coverage(path + '.yaml'):.1f}%")
            summary.append(f"지도: {path}.pgm")
        else:
            summary.append("지도 저장 실패")
        print("\n" + "\n".join(summary))
        log = os.path.join(REPO, "logs", f"explore_{stamp}")
        with open(log + ".txt", "w") as f:
            f.write("\n".join(summary) + "\n")
        with open(log + ".csv", "w") as f:
            f.write("t_s,true_x,true_y,true_yaw,est_x,est_y,est_yaw,err_m\n")
            for (t, tr, es), e in zip(samples, errs):
                f.write(f"{t:.1f},{tr[0]:.3f},{tr[1]:.3f},{tr[2]:.3f},{es[0]:.3f},{es[1]:.3f},{es[2]:.3f},{e:.3f}\n")
        print(f"기록: {log}.txt / .csv")
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
