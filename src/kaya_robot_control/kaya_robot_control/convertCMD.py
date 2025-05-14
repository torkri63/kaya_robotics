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
            '/planned_path',
            self.pose_callback,
            10
        )

        # Previous pose values
        self.prev_x = None
        self.prev_y = None
        self.prev_z = None

        self.linear_speed = 0.2
        self.angular_speed = 1.0

    def pose_callback(self, msg):
        # Extract position
        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z

        # Extract orientation
        ox = msg.pose.orientation.x
        oy = msg.pose.orientation.y
        oz = msg.pose.orientation.z
        ow = msg.pose.orientation.w

        # Log orientation values (optional)
        self.get_logger().info(f"Orientation: x={ox:.2f}, y={oy:.2f}, z={oz:.2f}, w={ow:.2f}")

        if self.prev_x is None or self.prev_y is None or self.prev_z is None:
            self.prev_x = x
            self.prev_y = y
            self.prev_z = z
            return

        dx = x - self.prev_x
        dy = y - self.prev_y
        dz = z - self.prev_z

        # Store current as previous for next callback
        self.prev_x = x
        self.prev_y = y
        self.prev_z = z

        # Threshold to avoid noise
        threshold = 0.01

        # Build command
        cmd = Twist()

        if abs(dx) > threshold:
            cmd.linear.x = self.linear_speed if dx > 0 else -self.linear_speed
        if abs(dy) > threshold:
            cmd.linear.y = self.linear_speed if dy > 0 else -self.linear_speed
        if abs(dz) > threshold:
            cmd.linear.z = self.linear_speed if dz > 0 else -self.linear_speed

        # Orientation could be processed into angular.z here if needed
        # For example, you could use a function that maps change in orientation to angular.z

        self.publisher_.publish(cmd)
        self.get_logger().info(
            f"Published cmd_vel: x={cmd.linear.x:.2f}, y={cmd.linear.y:.2f}, z={cmd.linear.z:.2f}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = AutoTeleopFromPose()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown() 

if __name__ == '__main__':
    main()
