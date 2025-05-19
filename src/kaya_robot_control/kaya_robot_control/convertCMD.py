#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
import math

class AutoTeleopFromPose(Node):
    def __init__(self):
        super().__init__('auto_teleop_from_pose')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(
            Path,
            '/planned_path',
            self.pose_callback,
            10
        )

        self.prev_x = 0.0
        self.prev_y = 0.0
        self.prev_z = 0.0

        self.linear_speed = 0.2  # Max linear speed
        self.angular_speed = 1.0  # Max angular speed
        self.max_distance = 2.0  # Maximum distance for scaling linear speed

        # Time of last processed callback
        self.last_processed_time = self.get_clock().now()

        # Interval in seconds
        self.processing_interval = 5.0

    def pose_callback(self, msg):
        now = self.get_clock().now()
        elapsed_time = (now - self.last_processed_time).nanoseconds / 1e9

        if elapsed_time < self.processing_interval:
            return  # Skip processing

        self.last_processed_time = now  # Update time of last processing

        if not msg.poses:
            self.get_logger().warn("Received empty path")
            return

        self.get_logger().info(f"Received path with {len(msg.poses)} poses")

        # Print all pose positions for debugging
        for i, pose_stamped in enumerate(msg.poses):
            pos = pose_stamped.pose.position
            self.get_logger().info(f"Pose {i}: x={pos.x:.2f}, y={pos.y:.2f}, z={pos.z:.2f}")

        # Use the last pose from the path
        current_pose = msg.poses[10].pose
        x = current_pose.position.x
        y = current_pose.position.y
        z = current_pose.position.z

        self.get_logger().info(f"Selected pose: x={x:.2f}, y={y:.2f}, z={z:.2f}")

        # Orientation
        ox = current_pose.orientation.x
        oy = current_pose.orientation.y
        oz = current_pose.orientation.z
        ow = current_pose.orientation.w
        self.get_logger().info(f"Orientation: x={ox:.2f}, y={oy:.2f}, z={oz:.2f}, w={ow:.2f}")

        goal_pose = msg.poses[0].pose
        xg = goal_pose.position.x
        yg = goal_pose.position.y
        zg = goal_pose.position.z

        # Calculate the difference in position
        dx = x - xg
        dy = y - yg
        dz = z - zg
      

        # Calculate the distance to the target pose
        distance = math.sqrt(dx**2 + dy**2 + dz**2)
        
        # Scale the linear speed based on distance to target
        scaled_linear_speed = self.linear_speed * min(distance / self.max_distance, 1.0)

        threshold = 0.01  # Movement threshold
        cmd = Twist()

        # Adjust the linear velocity
        if abs(dx) > threshold:
            cmd.linear.x = scaled_linear_speed if dx > 0 else -scaled_linear_speed
        if abs(dy) > threshold:
            cmd.linear.y = scaled_linear_speed if dy > 0 else -scaled_linear_speed
        if abs(dz) > threshold:
            cmd.linear.z = scaled_linear_speed if dz > 0 else -scaled_linear_speed

        # If the robot is close enough to the target, stop
        if distance < 0.1:
            cmd.linear.x = 0.0
            cmd.linear.y = 0.0
            cmd.linear.z = 0.0

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
