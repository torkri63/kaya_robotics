#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseArray, PoseStamped
from kaya_msgs.msg import CubeInfo



class MotionPlanningSubscriber(Node):
    """
    Subscribes to /cube/positions (PoseArray), republishes each detection as:
      - CubeInfo on /cube_info
      - PoseStamped on /goal_position (for MotionPlanner)

    Skips any TF transforms for now; will add later.
    """

    def __init__(self):
        super().__init__('motion_planning_subscriber')

        # Input topic for PoseArray of detected cubes
        self.declare_parameter('input_topic', '/cube/positions')
        self.input_topic = self.get_parameter('input_topic').value

        # Publisher for CubeInfo
        self.pub_info = self.create_publisher(CubeInfo, '/cube_info', 10)
        # Publisher for PoseStamped goal positions
        self.pub_goal = self.create_publisher(PoseStamped, 'goal_position', 10)

        # Subscribe to PoseArray
        self.sub = self.create_subscription(
            PoseArray,
            self.input_topic,
            self.on_pose_array,
            10
        )

        self.get_logger().info(
            f"Subscribed to '{self.input_topic}', will publish CubeInfo on /cube_info and PoseStamped on /goal_position"
        )

    def on_pose_array(self, msg: PoseArray):
        # Log received message for debugging
        self.get_logger().info(
            f"Received PoseArray with {len(msg.poses)} poses on {self.input_topic}"
        )

        # For each detected cube pose, publish both CubeInfo and PoseStamped
        for pose in msg.poses:
            # Create CubeInfo message
            info = CubeInfo()
            info.header.stamp = msg.header.stamp
            info.header.frame_id = msg.header.frame_id  # e.g. 'camera_link'
            info.point = pose.position
            info.detected = True
            self.pub_info.publish(info)

            # Create PoseStamped message
            ps = PoseStamped()
            ps.header.stamp = msg.header.stamp
            ps.header.frame_id = msg.header.frame_id
            ps.pose.position = pose.position
            ps.pose.orientation.w = 1.0  # Neutral orientation
            self.pub_goal.publish(ps)

        self.get_logger().info(
            f"Republished {len(msg.poses)} cubes as CubeInfo + PoseStamped goals"
        )


def main(args=None):
    rclpy.init(args=args)
    node = MotionPlanningSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

