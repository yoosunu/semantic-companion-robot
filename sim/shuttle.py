#!/usr/bin/env python3
"""왕복 도구: A ↔ B 를 계속 오가며, 매 구간마다 결과를 한 줄로 찍는다.

돌려두고 다른 창에서 Nav2 값을 바꾸면 → 다음 구간에서 차이가 보인다.
  robot go                     # 기본: A=(0,0,0°) ↔ B=(3,0,180°), 무한 반복
  robot go --laps 3           # 3번 왕복 후 요약 출력
  robot go --a 0,0,0 --b 2.5,0,180   # x,y,방향(도) — map 좌표
  시작하면 먼저 A로 간다 (준비 구간, 기록 안 함) → 로봇이 어디 있든 같은 조건에서 시작
  Ctrl+C: 지금 목표를 취소(로봇 정지) → 요약 출력

한 줄 예:
  [3 A→B] SUCCEEDED  17.9 s | 오차 xy 0.04 m, yaw 11.8° | 최고 v 0.30 m/s, w 0.98 rad/s
  - 오차는 Nav2가 믿는 위치(TF map→base_footprint) 기준 = 도착 판정과 같은 값. 실제(정답) 위치 비교는 robot test
  - 최고 v, w 는 /cmd_vel (브리지가 받는 최종 명령) 기준
  - 구간 시작 때 감시 중인 Nav2 값이 바뀌었으면 "params:" 줄을 찍는다 → 내 변경이 실제 적용됐는지 확인
기록: logs/shuttle_<시각>.csv

이 파일은 놀이용 도구다. Nav2에 목표를 보내는 진짜 로직(실패 처리, 판단)은 task_manager에서 Jasper가 짠다.
"""
import argparse
import csv
import math
import os
import time
from datetime import datetime

import rclpy
import rclpy.time
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import TwistStamped
from tf2_ros import Buffer, TransformListener
from nav2_msgs.action import NavigateToPose
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters
from lifecycle_msgs.srv import GetState

STATUS_NAME = {
    GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
    GoalStatus.STATUS_ABORTED: "ABORTED",
    GoalStatus.STATUS_CANCELED: "CANCELED",
}

# 구간마다 읽어서, 바뀌었으면 출력할 Nav2 값 (my_nav2.yaml 의 손잡이들)
WATCH = {
    "controller_server": [
        "goal_checker.xy_goal_tolerance",
        "goal_checker.yaw_goal_tolerance",
        "FollowPath.max_vel_x",
        "FollowPath.max_speed_xy",
        "FollowPath.max_vel_theta",
        "FollowPath.RotateToGoal.scale",
    ],
    "velocity_smoother": ["max_velocity"],
}


def parse_pose(text):
    x, y, yaw_deg = (float(v) for v in text.split(","))
    return x, y, math.radians(yaw_deg)


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def wrap(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


def param_value(pv):
    t = pv.type
    if t == ParameterType.PARAMETER_DOUBLE:
        return round(pv.double_value, 4)
    if t == ParameterType.PARAMETER_INTEGER:
        return pv.integer_value
    if t == ParameterType.PARAMETER_BOOL:
        return pv.bool_value
    if t == ParameterType.PARAMETER_STRING:
        return pv.string_value
    if t == ParameterType.PARAMETER_DOUBLE_ARRAY:
        return [round(v, 4) for v in pv.double_array_value]
    return None  # 없는 이름이면 NOT_SET


class Shuttle(Node):
    def __init__(self):
        super().__init__("shuttle")
        self.nav = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.param_clients = {n: self.create_client(GetParameters, f"/{n}/get_parameters") for n in WATCH}
        # 로봇 위치 = TF map→base_footprint (Nav2가 도착 판정에 쓰는 것과 같은 값)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(TwistStamped, "cmd_vel", self.on_cmd_vel, 10)
        self.bt_state = self.create_client(GetState, "/bt_navigator/get_state")
        self.goal_handle = None
        self.max_v = 0.0
        self.max_w = 0.0

    def robot_pose(self):
        """(x, y, yaw) in map. 없으면 None."""
        try:
            t = self.tf_buffer.lookup_transform("map", "base_footprint", rclpy.time.Time())
        except Exception:
            return None
        tr = t.transform.translation
        return tr.x, tr.y, yaw_of(t.transform.rotation)

    def on_cmd_vel(self, msg):
        self.max_v = max(self.max_v, abs(msg.twist.linear.x))
        self.max_w = max(self.max_w, abs(msg.twist.angular.z))

    def read_params(self):
        """감시 중인 Nav2 값을 {'노드/이름': 값} 으로. 응답 없으면 그 노드는 건너뜀."""
        values = {}
        for node, names in WATCH.items():
            client = self.param_clients[node]
            if not client.wait_for_service(timeout_sec=1.0):
                continue
            future = client.call_async(GetParameters.Request(names=names))
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
            if future.result() is None:
                continue
            for name, pv in zip(names, future.result().values):
                values[f"{node}/{name}"] = param_value(pv)
        return values

    def nav2_ready(self):
        """bt_navigator 가 active 면 True (Nav2가 목표를 받을 준비가 됨)."""
        if not self.bt_state.wait_for_service(timeout_sec=1.0):
            return False
        future = self.bt_state.call_async(GetState.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        return future.result() is not None and future.result().current_state.label == "active"

    def go(self, pose):
        """목표 하나를 보내고 끝날 때까지 기다린다. 반환: 결과 이름."""
        x, y, yaw = pose
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.z = math.sin(yaw / 2)
        goal.pose.pose.orientation.w = math.cos(yaw / 2)

        send = self.nav.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send)
        handle = send.result()
        if handle is None or not handle.accepted:
            return "REJECTED"
        self.goal_handle = handle
        result = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result)
        self.goal_handle = None
        status = result.result().status
        return STATUS_NAME.get(status, f"STATUS_{status}")

    def cancel(self):
        if self.goal_handle is None:
            return
        print("\n목표 취소 중...")
        future = self.goal_handle.cancel_goal_async()
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
        self.goal_handle = None

    def settle(self, seconds=0.5):
        """도착 직후 TF를 잠깐 받는다."""
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)


def main():
    ap = argparse.ArgumentParser(description="A ↔ B 왕복 (Nav2 놀이용)")
    ap.add_argument("--a", default="0,0,0", help="x,y,방향(도)  map 좌표 (기본 0,0,0 = 출발점)")
    ap.add_argument("--b", default="3.0,0,180", help="x,y,방향(도)  (기본 3.0,0,180 = 3 m 앞, 돌아서 섬)")
    ap.add_argument("--laps", type=int, default=0, help="왕복 횟수 (0 = Ctrl+C 까지)")
    args = ap.parse_args()
    points = {"A": parse_pose(args.a), "B": parse_pose(args.b)}

    # Ctrl+C 때 rclpy가 먼저 꺼지지 않게 → 우리가 목표를 취소하고 나서 끈다
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Shuttle()

    log_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "logs")
    os.makedirs(log_dir, exist_ok=True)
    csv_path = os.path.realpath(os.path.join(log_dir, f"shuttle_{datetime.now():%Y%m%d_%H%M%S}.csv"))
    csv_file = open(csv_path, "w", newline="")
    writer = csv.writer(csv_file)
    writer.writerow(["leg", "from", "to", "result", "time_s", "xy_err_m", "yaw_err_deg", "max_v", "max_w", "params"])

    rows = []
    try:
        print("Nav2 (navigate_to_pose) 기다리는 중...")
        # 시작 전 점검: ① Nav2가 목표를 받을 수 있나  ② 지도 위 로봇 위치(TF map→base_footprint)가 나오나
        #   ②가 없으면 AMCL이 아직 위치를 못 잡은 것 → 출발하면 헤맴
        start_wait = time.monotonic()
        while True:
            node.settle(1.0)
            nav_ok = node.nav.server_is_ready() and node.nav2_ready()
            pose_ok = node.robot_pose() is not None
            if nav_ok and pose_ok:
                break
            missing = ", ".join(m for m, ok in (("Nav2", nav_ok), ("지도 위 위치(AMCL)", pose_ok)) if not ok)
            print(f"  준비 안 됨: {missing} ({time.monotonic() - start_wait:.0f} s)... 오래 걸리면 robot status")
            time.sleep(1.0)
        print("  준비 완료")
        print(f"A={args.a}  B={args.b}  laps={args.laps or '무한'}  기록: {csv_path}\n")

        # 준비 구간: 로봇이 어디서 어떤 방향으로 있든 먼저 A로 간다 (기록·요약에서 제외)
        # → 이후 모든 구간은 같은 A→B / B→A 라서 서로 비교할 수 있다
        print("[준비] A로 이동 (기록 안 함)")
        result = node.go(points["A"])
        print(f"[준비] {result}\n")
        if result == "REJECTED":
            print("  → Nav2가 목표를 거절함. 멈춤 → robot status 로 확인")
            return

        last_params = None
        leg = 0
        here, there = "A", "B"
        while args.laps == 0 or leg < args.laps * 2:
            leg += 1
            params = node.read_params()
            if params != last_params:
                print("params: " + ", ".join(f"{k.split('/')[1]}={v}" for k, v in params.items()))
                last_params = params

            node.max_v = node.max_w = 0.0
            start = time.monotonic()
            result = node.go(points[there])
            elapsed = time.monotonic() - start
            node.settle()

            gx, gy, gyaw = points[there]
            xy_err = yaw_err = float("nan")
            pose = node.robot_pose()
            if pose is not None:
                xy_err = math.hypot(pose[0] - gx, pose[1] - gy)
                yaw_err = math.degrees(abs(wrap(pose[2] - gyaw)))

            print(f"[{leg} {here}→{there}] {result:9s} {elapsed:5.1f} s | "
                  f"오차 xy {xy_err:.2f} m, yaw {yaw_err:4.1f}° | 최고 v {node.max_v:.2f} m/s, w {node.max_w:.2f} rad/s")
            row = [leg, here, there, result, f"{elapsed:.1f}", f"{xy_err:.3f}", f"{yaw_err:.1f}",
                   f"{node.max_v:.2f}", f"{node.max_w:.2f}", params]
            writer.writerow(row)
            csv_file.flush()
            rows.append((result, elapsed, xy_err, yaw_err))

            if result == "REJECTED":
                print("  → Nav2가 목표를 거절함 (Nav2가 꺼졌거나 준비 안 됨). 멈춤 → robot status 로 확인")
                break
            if result != "SUCCEEDED":
                print("  → 실패/취소. 그래도 반대편으로 계속 간다 (멈추려면 Ctrl+C)")
            here, there = there, here
    except KeyboardInterrupt:
        node.cancel()
    finally:
        ok = [r for r in rows if r[0] == "SUCCEEDED"]
        print(f"\n요약: {len(rows)}구간, 성공 {len(ok)}")
        if ok:
            n = len(ok)
            print(f"  평균 시간 {sum(r[1] for r in ok) / n:.1f} s, "
                  f"평균 xy {sum(r[2] for r in ok) / n:.2f} m, 평균 yaw {sum(r[3] for r in ok) / n:.1f}°")
        print(f"기록: {csv_path}")
        csv_file.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
