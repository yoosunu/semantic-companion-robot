#!/usr/bin/env python3
"""Whisker (수염) — 기어다니는 로봇의 촉각. LiDAR(19.5 cm) 아래 것(의자 다리, 건조대 다리, 레일, 턱)을 닿아서 안다.

  느끼기   /imu + /motor_out(브리지가 방금 모터를 바꿨나 = efference copy) → 부딪힘 / 끄덕임 / 건드림
  방향     충격 방향 = 가속이 튄 방향의 반대 (회전 중 모서리, 옆 긁힘, 사람이 민 것도)
  반응     멈춤 → 왔던 쪽으로 10 cm 빼기 (회전 중이었으면 15° 되돌리기) → 성공하면 Nav2 에 돌려줌
  갇힘     빼다가 또 닿음 → 10초 가만히 → 다시 빼기 (1: 같은 방법, 2: 15° 돌고 빼기, 3: 반대로 15° 돌고 빼기)
           → 3번 다 실패하면 구조 요청 (ntfy) + 정지. 풀기: ros2 topic pub --once /whisker/resume std_msgs/msg/Empty
  기억     닿은 점을 장애물 지도에 (/whisker/cloud → costmap whisker_layer, LiDAR 가 못 지움). 좁게: 닿은 점 ~10 cm.
           지우기: ros2 topic pub --once /whisker/clear std_msgs/msg/Empty
  기록     ~/scr_logs/whisker/<날짜>.jsonl, 로그 줄 "SENSE {...}" (run_report 가 셈)

브리지는 얇게 (v8): /imu, /motor_out 을 내고, /whisker/cmd_vel 이 오는 동안은 Nav2 를 무시한다. 판단은 전부 여기.
문턱값·거리는 ROS 파라미터 → 재빌드 없이 `ros2 param set /whisker bump_g 0.05`. 측정 근거: docs/calibration/imu-touch-2026-11-06.md
축 방향: ax_forward / ay_left (+1/-1). 처음 한 번, 정면으로 박아서 dir 이 front 로 나오는지 확인 (아니면 ax_forward 뒤집기).
"""
import collections
import json
import math
import struct
import threading
import time
import urllib.request
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu, PointCloud2, PointField
from std_msgs.msg import Empty, Float32MultiArray, String

G = 9.81
HALF_L, HALF_W = 0.13, 0.12   # m, 차체 반길이·반폭 (footprint)
MARK_GAP = 0.03               # m, 몸 바깥으로
MARK_HALF = 0.05              # m, 표시 반폭 → 점 3개, 폭 ~10 cm
MARK_Z = 0.10                 # m, 장애물 높이 (costmap 높이 필터 안)
SELF_MASK = 0.4               # s, 우리가 모터를 바꾼 뒤 IMU 무시 (출발 흔들림 0.17 g 은 우리 것)
BACK_SPEED, TURN_SPEED = 0.05, 0.25   # m/s, rad/s (기어가기 바닥선 위)
STEP_TIMEOUT = 6.0            # s, 빼기 한 번 최대


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def side_name(ang):
    a = math.degrees(math.atan2(math.sin(ang), math.cos(ang)))
    for lo, hi, n in ((-45, 45, "front"), (45, 135, "left"), (-135, -45, "right")):
        if lo <= a < hi:
            return n
    return "rear"


class Whisker(Node):
    def __init__(self):
        super().__init__("whisker")
        p = self.declare_parameter
        self.bump_g = p("bump_g", 0.06).value          # 주행 중 수평 가속 튐 (상자 0.09, 주행 잡음 0.02)
        self.pitch_dps = p("pitch_dps", 5.5).value     # 주행 중 앞뒤 끄덕임 (카펫 7)
        self.touch_g = p("touch_g", 0.025).value       # 정지 중 건드림 (톡 0.04)
        self.rock_dps = p("rock_dps", 4.0).value       # 정지 중 기우뚱 (톡 6)
        self.back_dist = p("back_dist", 0.10).value    # m, 빼는 거리
        self.back_turn = math.radians(p("back_turn_deg", 15.0).value)
        self.wait_s = p("trapped_wait", 10.0).value
        self.ax_forward = p("ax_forward", 1.0).value   # 보드 +x 가 로봇 앞이면 +1 (첫 정면 박기로 확인)
        self.ay_left = p("ay_left", 1.0).value         # 보드 +y 가 로봇 왼쪽이면 +1
        self.react = p("react", True).value            # False = 느끼고 기록만 (관찰 모드)

        # 감각 상태
        self.base = None                     # [ax, ay, gx, gy] 평소 값 (천천히 따라감)
        self.motor = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.prev_motor = None
        self.last_self = 0.0                 # 우리가 마지막으로 모터를 바꾼 시각
        self.last_event = 0.0
        # 위치
        self.odom = None                     # (x, y, yaw) odom
        self.odom_hist = collections.deque(maxlen=20 * 180)
        self.map_fix = None
        # 반응 상태 기계
        self.state = "drive"                 # drive / stop / back / trapped / help
        self.state_t = 0.0
        self.step = None                     # 지금 빼기 계획 {"kind": "lin"/"rot", "sign", "goal", "start", "pre_rot"}
        self.contact_ang = 0.0               # 마지막 충격 방향 (몸 기준 rad, 0 = 앞, + = 왼쪽)
        self.last_cmd = (0.0, 0.0)           # 닿기 직전 우리가 따르던 명령 (x, z)
        self.trapped = 0
        self.left_hits, self.right_hits = 0, 0
        # 기억
        self.points = []
        self.contacts = 0

        self.create_subscription(Imu, "imu", self.on_imu, 50)
        self.create_subscription(Float32MultiArray, "motor_out", self.on_motor, 50)
        self.create_subscription(Odometry, "odom", self.on_odom, 20)
        for t in ("pose", "amcl_pose"):
            self.create_subscription(PoseWithCovarianceStamped, t, self.on_map_pose, 10)
        self.create_subscription(Empty, "whisker/clear", self.on_clear, 1)
        self.create_subscription(Empty, "whisker/resume", self.on_resume, 1)
        self.cmd_pub = self.create_publisher(TwistStamped, "whisker/cmd_vel", 10)
        self.cloud_pub = self.create_publisher(PointCloud2, "whisker/cloud", 1)
        self.sense_pub = self.create_publisher(String, "sense", 10)
        self.create_timer(0.05, self.tick)
        self.create_timer(1.0, self.publish_cloud)
        d = Path.home() / "scr_logs" / "whisker"
        d.mkdir(parents=True, exist_ok=True)
        self.log_path = d / f"{time.strftime('%Y-%m-%d')}.jsonl"
        self.get_logger().info(f"whisker ready (react={self.react}) → log {self.log_path}")

    # ------------------------------------------------------------------ 입력
    def on_motor(self, m):
        d = list(m.data) + [0.0] * 5
        self.motor = d[:5]
        if self.prev_motor is not None:
            pl, pr, cx, cz = self.prev_motor[:4]
            if (abs(d[0] - pl) > 8 or abs(d[1] - pr) > 8 or abs(d[2] - cx) > 0.02 or abs(d[3] - cz) > 0.05):
                self.last_self = time.time()
        self.prev_motor = d[:5]

    def on_odom(self, m):
        st, pp = m.header.stamp, m.pose.pose
        self.odom = (pp.position.x, pp.position.y, yaw_of(pp.orientation))
        self.odom_hist.append((st.sec + st.nanosec * 1e-9, *self.odom))

    def on_map_pose(self, m):
        st, pp = m.header.stamp, m.pose.pose
        self.map_fix = (st.sec + st.nanosec * 1e-9, pp.position.x, pp.position.y, yaw_of(pp.orientation))

    def on_imu(self, m):
        now = time.time()
        a, w = m.linear_acceleration, m.angular_velocity
        s = [a.x / G, a.y / G, math.degrees(w.x), math.degrees(w.y)]
        if self.base is None:
            self.base = s[:]
            return
        d = [s[i] - self.base[i] for i in range(4)]
        horiz = math.hypot(d[0], d[1])
        motors_on = abs(self.motor[0]) > 1 or abs(self.motor[1]) > 1
        settled = now - self.last_self > SELF_MASK
        if settled and horiz < self.touch_g and abs(d[2]) < self.rock_dps and abs(d[3]) < self.rock_dps:
            self.base = [self.base[i] + 0.02 * d[i] for i in range(4)]   # 평소 값은 조용할 때만 천천히
        if not settled or now - self.last_event < 1.0:
            return
        ev = None
        if motors_on and horiz > self.bump_g:
            ev = {"ev": "bump", "g": round(horiz, 3)}
        elif motors_on and abs(d[3]) > self.pitch_dps:
            ev = {"ev": "pitch", "dps": round(d[3], 1)}
        elif not motors_on and (horiz > self.touch_g or abs(d[2]) > self.rock_dps or abs(d[3]) > self.rock_dps):
            ev = {"ev": "touch", "g": round(horiz, 3), "gx": round(d[2], 1), "gy": round(d[3], 1)}
        if not ev:
            return
        self.last_event = now
        # 충격 방향 (몸 기준): 로봇은 닿은 쪽 반대로 밀린다 → 접촉 = 가속 튐의 반대
        bx, by = self.ax_forward * d[0], self.ay_left * d[1]
        if ev["ev"] == "pitch" or horiz < 1e-4:   # 끄덕임: 가던 쪽 (앞으로 가다 들리면 앞)
            ang = 0.0 if self.motor[2] >= 0 else math.pi
        else:
            ang = math.atan2(-by, -bx)
        ev.update(dir=side_name(ang), ang=round(math.degrees(ang)), ax=round(d[0], 3), ay=round(d[1], 3),
                  cmd_x=round(self.motor[2], 3), cmd_z=round(self.motor[3], 2), state=self.state)
        self.handle(ev, ang)

    # ------------------------------------------------------------------ 반응
    def handle(self, ev, ang):
        kind = ev["ev"]
        marked = kind in ("bump", "pitch") and self.mark(ang)
        ev["marked"] = marked
        self.record(ev)
        if kind == "touch" or not self.react:
            return
        if ang > 0.3:
            self.left_hits += 1
        elif ang < -0.3:
            self.right_hits += 1
        if self.state == "drive":
            self.contact_ang = ang
            self.last_cmd = (self.motor[2], self.motor[3])
            self.trapped = 0
            self.go("stop")
        elif self.state == "back":          # 빼다가 또 닿음 = 갇힘
            self.trapped += 1
            self.contact_ang = ang
            if self.trapped > 3:            # 시도 1·2·3 다 실패
                self.go("help")
            else:
                self.go("trapped")

    def go(self, state):
        self.state, self.state_t = state, time.time()
        if state == "back":
            self.plan_step()
        msg = {"stop": "멈춤", "back": "빼기", "trapped": f"갇힘 → {self.wait_s:.0f}초 대기 후 시도 {self.trapped}/3",
               "help": "구조 요청 — 3번 다 갇힘", "drive": "Nav2 에 돌려줌"}[state]
        self.get_logger().warn(f"WHISKER {state}: {msg}")
        if state == "help":
            where = self.map_pose()
            threading.Thread(target=self.ntfy, daemon=True, args=("robot: WHISKER 구조 요청",
                      f"앞뒤가 막혀 3번 빠져나오지 못함. 위치 {where[1]:.2f}, {where[2]:.2f} ({where[0]})" if where else
                      "앞뒤가 막혀 3번 빠져나오지 못함")).start()

    def plan_step(self):
        """빼기 계획. 처음·시도 1: 그대로 / 시도 2: 덜 박힌 쪽으로 15° 먼저 / 시도 3: 반대로 15° 먼저"""
        x0, z0 = self.last_cmd
        rotating = abs(x0) < 0.01 and abs(z0) > 0.05
        pre = 0.0
        if self.trapped == 2:
            pre = -self.back_turn if self.left_hits > self.right_hits else self.back_turn
        elif self.trapped >= 3:
            pre = self.back_turn if self.left_hits > self.right_hits else -self.back_turn
        if rotating:   # 돌다가 닿음 → 돌던 반대로 15°
            self.step = {"kind": "rot", "sign": -math.copysign(1, z0), "goal": self.back_turn, "pre": pre}
        else:          # 가다가 닿음 → 충격 반대쪽으로 직선 (앞에서 닿았으면 후진)
            sign = -1.0 if math.cos(self.contact_ang) > 0 else 1.0
            self.step = {"kind": "lin", "sign": sign, "goal": self.back_dist, "pre": pre}
        self.step["start"] = self.odom
        self.step["phase"] = "pre" if pre else "main"

    def tick(self):
        if self.state == "drive":
            return
        now, t = time.time(), time.time() - self.state_t
        x = z = 0.0
        if self.state == "stop":
            if t > 0.5:
                self.go("back")
        elif self.state == "trapped":
            if t > self.wait_s:
                self.go("back")
        elif self.state == "back":
            st = self.step
            if not self.odom or not st["start"]:
                self.go("drive")
                return
            x0, y0, yaw0 = st["start"]
            x1, y1, yaw1 = self.odom
            if st["phase"] == "pre":
                turned = abs(math.atan2(math.sin(yaw1 - yaw0), math.cos(yaw1 - yaw0)))
                if turned >= abs(st["pre"]):
                    st["phase"], st["start"] = "main", self.odom
                else:
                    z = math.copysign(TURN_SPEED, st["pre"])
            if st["phase"] == "main":
                if st["kind"] == "lin":
                    done = math.hypot(x1 - x0, y1 - y0) >= st["goal"]
                    x = 0.0 if done else st["sign"] * BACK_SPEED
                else:
                    done = abs(math.atan2(math.sin(yaw1 - yaw0), math.cos(yaw1 - yaw0))) >= st["goal"]
                    z = 0.0 if done else st["sign"] * TURN_SPEED
                if done:
                    self.trapped = 0
                    self.send(0.0, 0.0)
                    self.go("drive")
                    return
            if t > STEP_TIMEOUT:   # 안 움직임 (헛돎) → 갇힌 걸로
                self.trapped += 1
                self.go("help" if self.trapped > 3 else "trapped")
                return
        self.send(x, z)

    def send(self, x, z):
        m = TwistStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = "base_link"
        m.twist.linear.x, m.twist.angular.z = float(x), float(z)
        self.cmd_pub.publish(m)

    def on_resume(self, _):
        self.trapped = 0
        self.go("drive")

    # ------------------------------------------------------------------ 기억
    def map_pose(self):
        if not self.odom_hist:
            return None
        _, ox, oy, oyaw = self.odom_hist[-1]
        if not self.map_fix:
            return "odom", ox, oy, oyaw
        ts, mx, my, myaw = self.map_fix
        old = min(self.odom_hist, key=lambda r: abs(r[0] - ts))
        dx, dy = ox - old[1], oy - old[2]
        c, s = math.cos(-old[3]), math.sin(-old[3])
        lx, ly = c * dx - s * dy, s * dx + c * dy
        c, s = math.cos(myaw), math.sin(myaw)
        return "map", mx + c * lx - s * ly, my + s * lx + c * ly, myaw + (oyaw - old[3])

    def mark(self, ang):
        """충격 방향의 몸 둘레 점 + 3 cm 바깥에, 방향과 직각으로 점 3개 (~10 cm)"""
        pose = self.map_pose()
        if not pose or pose[0] != "map":
            return False
        _, x, y, yaw = pose
        ca, sa = math.cos(ang), math.sin(ang)
        r = min(HALF_L / abs(ca) if abs(ca) > 1e-6 else 9, HALF_W / abs(sa) if abs(sa) > 1e-6 else 9) + MARK_GAP
        for off in (-MARK_HALF, 0.0, MARK_HALF):
            bx, by = r * ca - off * sa, r * sa + off * ca            # 몸 기준
            self.points.append((x + math.cos(yaw) * bx - math.sin(yaw) * by,
                                y + math.sin(yaw) * bx + math.cos(yaw) * by))
        self.contacts += 1
        self.publish_cloud()
        return True

    def record(self, ev):
        pose = self.map_pose()
        if pose:
            ev.update(frame=pose[0], x=round(pose[1], 3), y=round(pose[2], 3), yaw=round(pose[3], 3))
        ev["t"] = round(time.time(), 2)
        with open(self.log_path, "a") as f:
            f.write(json.dumps(ev, ensure_ascii=False) + "\n")
        self.get_logger().warn("SENSE " + json.dumps(ev, ensure_ascii=False))
        self.sense_pub.publish(String(data=json.dumps(ev, ensure_ascii=False)))

    def on_clear(self, _):
        self.points.clear()
        self.contacts = 0
        self.get_logger().warn("WHISKER clear: 표시 전부 지움")
        self.publish_cloud()

    def publish_cloud(self):
        m = PointCloud2()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = "map"
        m.height, m.width = 1, len(self.points)
        m.fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1) for i, n in enumerate("xyz")]
        m.is_bigendian, m.point_step, m.is_dense = False, 12, True
        m.row_step = 12 * len(self.points)
        m.data = b"".join(struct.pack("<fff", px, py, MARK_Z) for px, py in self.points)
        self.cloud_pub.publish(m)

    @staticmethod
    def ntfy(title, body):
        env = Path.home() / ".scr.env"
        topic = None
        if env.exists():
            for ln in env.read_text().splitlines():
                if ln.startswith("NTFY_TOPIC="):
                    topic = ln.split("=", 1)[1].strip()
        if not topic:
            return
        try:
            urllib.request.urlopen(urllib.request.Request("https://ntfy.sh/" + topic, data=body.encode(), headers={
                "Title": title, "Priority": "high", "Tags": "warning"}), timeout=10)
        except Exception:
            pass


def main():
    rclpy.init()
    n = Whisker()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
