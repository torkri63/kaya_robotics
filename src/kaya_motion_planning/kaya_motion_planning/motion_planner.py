import rclpy
from rclpy.node import Node
from std_msgs.msg import String                            # ⬅️ for state gating
from geometry_msgs.msg import Pose
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import numpy as np

class MotionPlanner(Node):
    def __init__(self):
        super().__init__('motion_planner')

        # Track state-machine state
        self.current_state = None
        self.create_subscription(
            String,
            'current_state',
            self.state_callback,
            10
        )

        self.robot_position = None
        self.goal_position = None

        # Subscribers for positions
        self.create_subscription(Pose, 'robot_position', self.robot_position_callback, 10)
        self.create_subscription(Pose, 'goal_position', self.goal_position_callback, 10)

        # Publisher for planned path
        self.path_publisher = self.create_publisher(Path, 'planned_path', 10)

        # Timer to trigger planning at 1 Hz
        self.timer = self.create_timer(1.0, self.plan_trajectory)

    def state_callback(self, msg: String):
        """Update the current state from the state machine."""
        self.current_state = msg.data
        self.get_logger().debug(f'Current state: {self.current_state}')

    def robot_position_callback(self, msg: Pose):
        self.robot_position = msg

    def goal_position_callback(self, msg: Pose):
        self.goal_position = msg

    def plan_trajectory(self):
        # Only plan during the EXECUTION phase
        if self.current_state != 'EXECUTION':
            return

        if self.robot_position is None or self.goal_position is None:
            return

        # Timestamp for all trajectory messages
        now = self.get_clock().now().to_msg()

        # Linear interpolation between start and goal
        x_start = self.robot_position.position.x
        y_start = self.robot_position.position.y
        x_goal = self.goal_position.position.x
        y_goal = self.goal_position.position.y

        num_points = 10
        x_values = np.linspace(x_start, x_goal, num_points)
        y_values = np.linspace(y_start, y_goal, num_points)

        path = Path()
        path.header.stamp = now
        path.header.frame_id = "map"

        for x, y in zip(x_values, y_values):
            pose = PoseStamped()
            pose.header.stamp = now
            pose.header.frame_id = "map"
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            path.poses.append(pose)

        self.path_publisher.publish(path)
        self.get_logger().info('Published planned_path')

def main(args=None):
    rclpy.init(args=args)
    planner = MotionPlanner()
    rclpy.spin(planner)
    planner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
