#!/usr/bin/env python3

import sys
import time
import termios
import tty

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped

LINEAR_SPEED = 0.30
ANGULAR_SPEED = 1.0
MOVE_TIME = 0.40

class SafeTeleop(Node):
    def __init__(self):
        super().__init__('safe_teleop')

        self.publisher = self.create_publisher(
            TwistStamped,
            '/cmd_vel',
            10
        )

    def move(self, linear=0.0, angular=0.0):
        msg = TwistStamped()

        msg.header.stamp = self.get_clock().now().to_msg()

        msg.twist.linear.x = linear
        msg.twist.angular.z = angular

        end_time = time.monotonic() + MOVE_TIME

        while time.monotonic() < end_time:
            msg.header.stamp = self.get_clock().now().to_msg()
            self.publisher.publish(msg)
            time.sleep(0.05)

        self.stop()

    def stop(self):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()

        self.publisher.publish(msg)


def get_key():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)

    try:
        tty.setraw(fd)
        key = sys.stdin.read(1)

    finally:
        termios.tcsetattr(
            fd,
            termios.TCSADRAIN,
            old_settings
        )

    return key


def main():
    rclpy.init()

    node = SafeTeleop()

    print("""
Safe Teleop

w : forward
s : backward
a : turn left
d : turn right

space : stop
q : quit
""")

    try:
        while True:
            key = get_key()

            if key == 'w':
                node.move(linear=LINEAR_SPEED)

            elif key == 's':
                node.move(linear=-LINEAR_SPEED)

            elif key == 'a':
                node.move(angular=ANGULAR_SPEED)

            elif key == 'd':
                node.move(angular=-ANGULAR_SPEED)

            elif key == ' ':
                node.stop()

            elif key == 'q':
                node.stop()
                break

    except KeyboardInterrupt:
        node.stop()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
