#!/usr/bin/env python3
"""fake_esp32 — pretends to be the UGV02's ESP32 firmware, but drives the Gazebo robot.

    ugv02_bridge ──JSON line──> [virtual serial /tmp/ugv02_esp32] ──> fake_esp32 ──/sim/cmd_vel──> Gazebo TB3

Mimics what docs/ugv02-protocol.md says the real firmware does:
  - {"T":13,"X":<m/s>,"Z":<rad/s>}   ROS control          -> drive
  - {"T":1,"L":<m/s>,"R":<m/s>}       wheel speed control  -> drive
  - wheel speed limit ±0.5 m/s
  - no movement command for 3 s -> stop (firmware heartbeat)
  - {"T":131,"cmd":1}                 start streaming feedback (OFF by default, like the real board)
  - {"T":142,"cmd":<ms>}              feedback period (default 50 ms)
  - feedback line: real UGV02 format {"T":1001,"L","R","ax..gz","odl","odr","v"} (see protocol doc "Real feedback")
    L/R come from Gazebo's measured motion (/sim/odom), not from the command -> slip/lag show up like real.
    v: fake battery, 12.3 V draining slowly (faster while moving).
Prints every command change, so you can see exactly what the bridge sent (debug layer by layer).

Run via `robot up` (scripts/robot). Needs: sudo apt install python3-serial socat
"""
import argparse
import json
import math
import random
import time

import rclpy
import serial
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node

WHEEL_LIMIT = 0.5        # m/s per wheel (UGV02 wiki)
TRACK_WIDTH = 0.160      # m, TB3 burger wheel separation (stand-in for UGV02)
GYRO_BIAS = -15          # raw gz at rest, like the real board (10-31: −14.8 … −15.0)
FIRMWARE_TIMEOUT = 3.0   # s (UGV02 wiki)
BATTERY_START = 12.3     # V, full 3S pack


class FakeEsp32(Node):
    def __init__(self, port):
        super().__init__('fake_esp32')
        self.ser = serial.Serial(port, 115200, timeout=0)
        self.pub = self.create_publisher(TwistStamped, '/sim/cmd_vel', 10)
        self.buf = b''
        self.x = 0.0
        self.z = 0.0
        self.last_cmd = time.monotonic()   # firmware runs on its own clock, not sim time
        self.stopped = True
        # feedback (T:1001)
        self.feedback_on = False
        self.feedback_period = 0.05
        self.last_feedback = 0.0
        self.next_feedback = 0.0
        self.meas_v = 0.0          # measured by Gazebo
        self.meas_w = 0.0
        self.odl = 0.0           # cumulative wheel distance, m (sent as cm)
        self.odr = 0.0
        self.battery = BATTERY_START
        self.create_subscription(Odometry, '/sim/odom', self.on_sim_odom, 10)
        self.create_timer(0.02, self.poll_serial)   # 50 Hz
        self.create_timer(0.05, self.drive)         # 20 Hz
        self.create_timer(0.01, self.send_feedback) # checks every 10 ms, sends every feedback_period
        self.get_logger().info(f'listening on {port} (115200). waiting for JSON lines...')

    def poll_serial(self):
        data = self.ser.read(self.ser.in_waiting or 1)
        if not data:
            return
        self.buf += data
        while b'\n' in self.buf:
            line, self.buf = self.buf.split(b'\n', 1)
            self.handle_line(line.decode('utf-8', errors='replace').strip())
        if len(self.buf) > 4096:   # no newline for a long time -> garbage
            self.get_logger().warn('dropping 4 KB without newline (missing \\n at end of JSON?)')
            self.buf = b''

    def handle_line(self, line):
        if not line:
            return
        try:
            msg = json.loads(line)
            t = msg['T']
        except (json.JSONDecodeError, KeyError, TypeError) as e:
            self.get_logger().warn(f'bad line {line!r}: {e}')
            return

        if t == 131:
            self.feedback_on = bool(msg.get('cmd', 0))
            self.get_logger().info(f'recv {line}  ->  feedback stream {"ON" if self.feedback_on else "OFF"}')
            return
        if t == 142:
            self.feedback_period = max(0.01, float(msg.get('cmd', 50)) / 1000.0)
            self.get_logger().info(f'recv {line}  ->  feedback every {self.feedback_period * 1000:.0f} ms')
            return
        if t == 13:
            x, z = float(msg.get('X', 0.0)), float(msg.get('Z', 0.0))
        elif t == 1:
            left, right = float(msg.get('L', 0.0)), float(msg.get('R', 0.0))
            x, z = (left + right) / 2.0, (right - left) / TRACK_WIDTH
        else:
            self.get_logger().info(f'ignored (not a movement command): {line}')
            return

        x, z = self.limit_wheels(x, z)
        if (x, z) != (self.x, self.z) or self.stopped:
            self.get_logger().info(f'recv {line}  ->  drive x={x:+.3f} m/s z={z:+.3f} rad/s')
        self.x, self.z = x, z
        self.last_cmd = time.monotonic()
        self.stopped = False

    @staticmethod
    def limit_wheels(x, z):
        """Scale (x, z) down so that neither wheel exceeds WHEEL_LIMIT."""
        left = x - z * TRACK_WIDTH / 2.0
        right = x + z * TRACK_WIDTH / 2.0
        peak = max(abs(left), abs(right))
        if peak > WHEEL_LIMIT:
            s = WHEEL_LIMIT / peak
            return x * s, z * s
        return x, z

    def on_sim_odom(self, msg):
        self.meas_v = msg.twist.twist.linear.x
        self.meas_w = msg.twist.twist.angular.z

    def send_feedback(self):
        now = time.monotonic()
        if not self.feedback_on:
            return
        if self.last_feedback == 0.0:
            self.next_feedback = now
        if now < self.next_feedback:
            return
        # Fixed schedule (next += period), like a firmware timer. "send when >= period since last" would
        # drift to period + timer tick (~56 ms instead of 50 ms) -- measured as 17.7 Hz on 2026-10-19.
        self.next_feedback += self.feedback_period
        if self.next_feedback < now:          # fell far behind (VM hiccup): resync instead of bursting
            self.next_feedback = now + self.feedback_period
        dt = now - self.last_feedback if self.last_feedback else 0.0
        self.last_feedback = now
        moving = abs(self.meas_v) > 1e-3 or abs(self.meas_w) > 1e-3
        self.battery -= dt * (0.0005 if moving else 0.0001)     # V per second, just for testing
        left = self.meas_v - self.meas_w * TRACK_WIDTH / 2.0
        right = self.meas_v + self.meas_w * TRACK_WIDTH / 2.0
        # Real UGV02 format (measured 10-29/10-31): odl/odr cumulative cm (int), gz raw (16.4 units = 1 °/s)
        # with a bias like the real board (−15) + noise, v in centivolts. Gyro = Gazebo's TRUE turn rate
        # (no slip), wheels = left/right speeds → the sim now has the same "wheels vs gyro" choice as the robot.
        self.odl += left * dt
        self.odr += right * dt
        gz = round(math.degrees(self.meas_w) * 16.4 + GYRO_BIAS + random.gauss(0.0, 2.0))
        line = json.dumps({'T': 1001, 'L': round(left, 4), 'R': round(right, 4),
                           'ax': 0, 'ay': 0, 'az': 8500, 'gx': 0, 'gy': 0, 'gz': gz,
                           'odl': round(self.odl * 100), 'odr': round(self.odr * 100),
                           'v': round(self.battery * 100)}, separators=(',', ':'))
        self.ser.write((line + '\n').encode())

    def drive(self):
        if not self.stopped and time.monotonic() - self.last_cmd > FIRMWARE_TIMEOUT:
            if self.x == 0.0 and self.z == 0.0:
                # Already standing still (the bridge sent a stop): nothing to rescue, stay quiet.
                self.get_logger().info('no command for 3 s (already stopped)')
            else:
                # Robot was MOVING and commands vanished: this is the dangerous case the bridge should prevent.
                self.get_logger().warn('no movement command for 3 s while MOVING -> firmware stop')
            self.x = self.z = 0.0
            self.stopped = True
        out = TwistStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'base_link'
        out.twist.linear.x = self.x
        out.twist.angular.z = self.z
        self.pub.publish(out)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--port', default='/tmp/ugv02_esp32')
    args, ros_args = ap.parse_known_args()
    rclpy.init(args=ros_args)
    node = FakeEsp32(args.port)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
