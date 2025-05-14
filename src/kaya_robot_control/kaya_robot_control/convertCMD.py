#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
import math

class AutoTeleopFromPose(Node):
    def __init__(self):
        super().__init__('auto_teleop_from_pose')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(
            PoseStamped,
            '/planned_path',  # <-- Replace with your actual topic name
            self.pose_callback,
            10
        )

        self.prev_x = None
        self.prev_y = None
        self.linear_speed = 0.2
        self.angular_speed = 1.0

    def pose_callback(self, msg):
        x = msg.pose.position.x
        y = msg.pose.position.y

        if self.prev_x is None or self.prev_y is None:
            self.prev_x = x
            self.prev_y = y
            return

        dx = x - self.prev_x
        dy = y - self.prev_y

        # Store current as previous for next comparison
        self.prev_x = x
        self.prev_y = y

        # Compute movement command
        cmd = Twist()

        # Threshold to avoid noise
        threshold = 0.01

        if abs(dx) > abs(dy):  # Primarily x-direction
            if dx > threshold:
                cmd.linear.x = self.linear_speed  # 'w'
            elif dx < -threshold:
                cmd.linear.x = -self.linear_speed  # 's'
        else:  # Primarily y-direction
            if dy > threshold:
                cmd.linear.y = self.linear_speed  # 'a'
            elif dy < -threshold:
                cmd.linear.y = -self.linear_speed  # 'd'


        self.publisher_.publish(cmd)
        self.get_logger().info(f"Published cmd_vel: x={cmd.linear.x}, y={cmd.linear.y}")

def main(args=None):
    rclpy.init(args=args)
    node = AutoTeleopFromPose()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
