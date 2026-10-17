#!/usr/bin/env python3
"""fake_esp32 — pretends to be the UGV02's ESP32 firmware, but drives the Gazebo robot.

    ugv02_bridge ──JSON line──> [virtual serial /tmp/ugv02_esp32] ──> fake_esp32 ──/sim/cmd_vel──> Gazebo TB3

Mimics what docs/ugv02-protocol.md says the real firmware does:
  - {"T":13,"X":<m/s>,"Z":<rad/s>}   ROS control          -> drive
  - {"T":1,"L":<m/s>,"R":<m/s>}       wheel speed control  -> drive
  - wheel speed limit ±0.5 m/s
  - no movement command for 3 s -> stop (firmware heartbeat)
Prints every command change, so you can see exactly what the bridge sent (debug layer by layer).
Feedback (T:1001 -> bridge /odom) comes in bridge v1.

Run via scripts/ugv02_sim.sh. Needs: sudo apt install python3-serial socat
"""
import argparse
import json
import time

import rclpy
import serial
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node

WHEEL_LIMIT = 0.5        # m/s per wheel (UGV02 wiki)
TRACK_WIDTH = 0.160      # m, TB3 burger wheel separation (stand-in for UGV02)
FIRMWARE_TIMEOUT = 3.0   # s (UGV02 wiki)


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
        self.create_timer(0.02, self.poll_serial)   # 50 Hz
        self.create_timer(0.05, self.drive)         # 20 Hz
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
