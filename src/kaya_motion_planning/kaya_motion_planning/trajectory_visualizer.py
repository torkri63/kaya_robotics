import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
import matplotlib.pyplot as plt

class TrajectoryVisualizer(Node):
    def __init__(self):
        super().__init__('trajectory_visualizer')
        self.subscription = self.create_subscription(
            Path,
            'planned_path',
            self.path_callback,
            10
        )
        self.path_received = False

    def path_callback(self, msg):
        if not self.path_received:
            x_coords = [pose.pose.position.x for pose in msg.poses]
            y_coords = [pose.pose.position.y for pose in msg.poses]

            plt.figure()
            plt.plot(x_coords, y_coords, marker='o', label='Trajectory')
            plt.scatter(x_coords[0], y_coords[0], color='green', label='Start')
            plt.scatter(x_coords[-1], y_coords[-1], color='red', label='Goal')
            plt.xlabel('X')
            plt.ylabel('Y')
            plt.title('Planned Trajectory')
            plt.legend()
            plt.grid(True)
            plt.show()

            self.path_received = True  # Stop after the first trajectory is received

def main(args=None):
    rclpy.init(args=args)
    visualizer = TrajectoryVisualizer()
    rclpy.spin(visualizer)
    visualizer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
