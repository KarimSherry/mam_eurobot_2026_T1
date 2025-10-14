#!/usr/bin/env python3
"""Simple keyboard teleop for ROS2 publishing to /cmd_vel"""
import sys
import termios
import tty
import select
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_keyboard')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info('Teleop node started. Use arrow keys to move, q to quit')
        self.speed = 0.3
        self.turn = 1.0

    def publish_twist(self, linear_x=0.0, angular_z=0.0):
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        self.pub.publish(msg)


def get_key(timeout=0.1):
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        rlist, _, _ = select.select([fd], [], [], timeout)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return key


def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()
    try:
        while rclpy.ok():
            key = get_key()
            if key == '\x1b':  # start of escape sequence
                # read two more chars
                k2 = get_key()
                k3 = get_key()
                seq = k2 + k3
                if seq == '[A':  # up
                    node.publish_twist(node.speed, 0.0)
                elif seq == '[B':  # down
                    node.publish_twist(-node.speed, 0.0)
                elif seq == '[C':  # right
                    node.publish_twist(0.0, -node.turn)
                elif seq == '[D':  # left
                    node.publish_twist(0.0, node.turn)
            elif key == 'q':
                break
            else:
                # stop on any other key or timeout
                node.publish_twist(0.0, 0.0)
            rclpy.spin_once(node, timeout_sec=0)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_twist(0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
