import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import numpy as np

class MotionPlanner(Node):
    def __init__(self):
        super().__init__('motion_planner')
        self.robot_position = None
        self.goal_position = None

        # Subscribers
        self.create_subscription(Pose, 'robot_position', self.robot_position_callback, 10)
        self.create_subscription(Pose, 'goal_position', self.goal_position_callback, 10)

        # Publisher
        self.path_publisher = self.create_publisher(Path, 'planned_path', 10)

        # Timer to trigger planning
        self.timer = self.create_timer(1.0, self.plan_trajectory)

    def robot_position_callback(self, msg):
        self.robot_position = msg

    def goal_position_callback(self, msg):
        self.goal_position = msg

    def plan_trajectory(self):
        if self.robot_position is None or self.goal_position is None:
            return

        # Example: Simple linear interpolation between robot and goal positions
        x_start = self.robot_position.position.x
        y_start = self.robot_position.position.y
        x_goal = self.goal_position.position.x
        y_goal = self.goal_position.position.y

        num_points = 10  # Number of points in the trajectory
        x_values = np.linspace(x_start, x_goal, num_points)
        y_values = np.linspace(y_start, y_goal, num_points)

        path = Path()
        path.header.frame_id = "map"
        for x, y in zip(x_values, y_values):
            pose = PoseStamped()
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            path.poses.append(pose)

        self.path_publisher.publish(path)

def main(args=None):
    rclpy.init(args=args)
    planner = MotionPlanner()
    rclpy.spin(planner)
    planner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
