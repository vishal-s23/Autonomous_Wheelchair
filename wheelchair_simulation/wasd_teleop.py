#!/usr/bin/env python3

import sys
import termios
import tty
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class WASDTeleop(Node):
    def __init__(self):
        super().__init__('wasd_teleop')
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self.linear_speed = 1.0     # m/s
        self.angular_speed = 1.2    # rad/s

        self.get_logger().info(
            'WASD control:\n'
            '  W/S: forward/back\n'
            '  A/D: turn left/right\n'
            '  X  : stop\n'
            'CTRL+C to quit'
        )

        self.run()

    def get_key(self):
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)
        return ch

    def run(self):
        while rclpy.ok():
            key = self.get_key()
            twist = Twist()

            if key.lower() == 'w':
                twist.linear.x = self.linear_speed
            elif key.lower() == 's':
                twist.linear.x = -self.linear_speed
            elif key.lower() == 'a':
                twist.angular.z = self.angular_speed
            elif key.lower() == 'd':
                twist.angular.z = -self.angular_speed
            elif key.lower() == 'x':
                pass  # stop
            else:
                continue

            self.pub.publish(twist)

def main():
    rclpy.init()
    node = WASDTeleop()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
