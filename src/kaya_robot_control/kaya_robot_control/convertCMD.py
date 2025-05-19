#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from rclpy.time import Time

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

        self.linear_speed = 0.2
        self.angular_speed = 1.0

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

        # Use a pose that's not at the origin (e.g., the last one for now)
        current_pose = msg.poses[-1].pose
        x = current_pose.position.x
        y = current_pose.position.y
        z = current_pose.position.z

        self.get_logger().info(f"Selected pose: x={x:.2f}, y={y:.2f}, z={z:.2f}")


        # Use the first pose in the path
        #current_pose = msg.poses[0].pose

        #x = current_pose.position.x
        #y = current_pose.position.y
        #z = current_pose.position.z

        #self.get_logger().info(f"Received position: x={x:.2f}, y={y:.2f}, z={z:.2f}")

        ox = current_pose.orientation.x
        oy = current_pose.orientation.y
        oz = current_pose.orientation.z
        ow = current_pose.orientation.w

        self.get_logger().info(f"Orientation: x={ox:.2f}, y={oy:.2f}, z={oz:.2f}, w={ow:.2f}")

        #if self.prev_x is None or self.prev_y is None or self.prev_z is None:
            #self.prev_x = x
            #self.prev_y = y
            #self.prev_z = z
            #return

        dx = x - self.prev_x
        dy = y - self.prev_y
        dz = z - self.prev_z

        self.prev_x = x
        self.prev_y = y
        self.prev_z = z

        threshold = 0.01
        cmd = Twist()

        if abs(dx) > threshold:
            cmd.linear.x = self.linear_speed if dx > 0 else -self.linear_speed
        if abs(dy) > threshold:
            cmd.linear.y = self.linear_speed if dy > 0 else -self.linear_speed
        if abs(dz) > threshold:
            cmd.linear.z = self.linear_speed if dz > 0 else -self.linear_speed

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
