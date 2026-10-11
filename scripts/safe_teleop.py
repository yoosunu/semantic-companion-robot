#!/usr/bin/env python3
"""Hold-to-drive keyboard teleop with acceleration limits.

w/s : forward / backward     a/d : turn left / right
space : immediate stop        q : quit

Behaviour
- Publishes geometry_msgs/TwistStamped on /cmd_vel at CONTROL_HZ (the Gazebo bridge expects TwistStamped).
- Moves only while a key is held. Terminals don't report key release, so a key counts as
  "released" when no repeat arrives for HOLD_TIMEOUT. It must be longer than the OS
  key-repeat start delay (typically 0.25-0.5 s), otherwise motion stutters right after the first press.
- All pending key presses are drained every cycle and only the latest one is used,
  so held keys never queue up and keep the robot moving after release.
- Velocity ramps toward the target with MAX_LIN_ACC / MAX_ANG_ACC (no step changes).
- Space bypasses the ramp and stops immediately.
"""
import os
import select
import sys
import termios
import time
import tty

import rclpy
from geometry_msgs.msg import TwistStamped

LINEAR_SPEED = 0.20     # m/s   (TurtleBot3 Burger max 0.22)
ANGULAR_SPEED = 1.0     # rad/s (Burger max 2.84)
MAX_LIN_ACC = 0.5       # m/s^2
MAX_ANG_ACC = 2.0       # rad/s^2
CONTROL_HZ = 20.0
HOLD_TIMEOUT = 0.5      # s without key repeat -> treat as released
ZERO_REPEATS = 5        # after reaching zero, publish a few more zero commands, then go quiet

KEY_TARGETS = {
    'w': (LINEAR_SPEED, 0.0),
    's': (-LINEAR_SPEED, 0.0),
    'a': (0.0, ANGULAR_SPEED),
    'd': (0.0, -ANGULAR_SPEED),
}

HELP = """
Safe Teleop (hold to drive)
  w/s : forward / backward    a/d : turn left / right
  space : immediate stop      q : quit
"""


def ramp(current, target, max_step):
    """Move current toward target by at most max_step."""
    if target > current:
        return min(target, current + max_step)
    return max(target, current - max_step)


def read_pending_keys(fd):
    """Return every character currently waiting on stdin (non-blocking)."""
    keys = ''
    while select.select([fd], [], [], 0)[0]:
        ch = os.read(fd, 1).decode(errors='ignore')
        if not ch:
            break
        keys += ch
    return keys


def main():
    rclpy.init()
    node = rclpy.create_node('safe_teleop')
    pub = node.create_publisher(TwistStamped, '/cmd_vel', 10)

    def publish(lin, ang):
        msg = TwistStamped()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.twist.linear.x = lin
        msg.twist.angular.z = ang
        pub.publish(msg)

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    print(HELP)

    dt = 1.0 / CONTROL_HZ
    target_lin = target_ang = 0.0
    lin = ang = 0.0
    last_key_time = 0.0
    zero_sent = ZERO_REPEATS

    try:
        tty.setcbreak(fd)   # unbuffered keys, but Ctrl+C still raises KeyboardInterrupt
        next_tick = time.monotonic()
        while True:
            now = time.monotonic()
            keys = read_pending_keys(fd)

            if 'q' in keys:
                break
            if ' ' in keys:                       # emergency stop: bypass the ramp
                target_lin = target_ang = lin = ang = 0.0
                last_key_time = 0.0
                zero_sent = 0
            else:
                drive_keys = [k for k in keys if k in KEY_TARGETS]
                if drive_keys:                    # latest key wins; older queued presses are dropped
                    target_lin, target_ang = KEY_TARGETS[drive_keys[-1]]
                    last_key_time = now

            if now - last_key_time > HOLD_TIMEOUT:
                target_lin = target_ang = 0.0     # key released

            lin = ramp(lin, target_lin, MAX_LIN_ACC * dt)
            ang = ramp(ang, target_ang, MAX_ANG_ACC * dt)

            moving = lin != 0.0 or ang != 0.0
            if moving:
                zero_sent = 0
            if moving or zero_sent < ZERO_REPEATS:
                publish(lin, ang)
                if not moving:
                    zero_sent += 1

            sys.stdout.write(f"\r  linear {lin:+.2f} m/s   angular {ang:+.2f} rad/s   ")
            sys.stdout.flush()

            next_tick += dt
            time.sleep(max(0.0, next_tick - time.monotonic()))
    except KeyboardInterrupt:
        pass
    finally:
        for _ in range(3):
            publish(0.0, 0.0)
            time.sleep(0.02)
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        print("\nstopped.")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
