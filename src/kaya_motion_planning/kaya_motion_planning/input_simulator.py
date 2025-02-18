import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose

class InputSimulator(Node):
    def __init__(self):
        super().__init__('input_simulator')
        self.robot_position_publisher = self.create_publisher(Pose, 'robot_position', 10)
        self.goal_position_publisher = self.create_publisher(Pose, 'goal_position', 10)
        self.timer = self.create_timer(1.0, self.publish_positions)

    def publish_positions(self):
        # Simulate robot position
        robot_pose = Pose()
        robot_pose.position.x = 0.0  # Example: Robot starts at origin
        robot_pose.position.y = 0.0
        robot_pose.position.z = 0.0
        self.robot_position_publisher.publish(robot_pose)

        # Simulate goal position
        goal_pose = Pose()
        goal_pose.position.x = 5.0   # Example: Goal is at (5, 5)
        goal_pose.position.y = 5.0
        goal_pose.position.z = 0.0
        self.goal_position_publisher.publish(goal_pose)

def main(args=None):
    rclpy.init(args=args)
    simulator = InputSimulator()
    rclpy.spin(simulator)
    simulator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
