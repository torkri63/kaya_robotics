import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import numpy as np

class MotionPlanner(Node):
    def __init__(self):
        super().__init__('motion_planner')

        # Track the state-machine state
        self.current_state = None
        self.create_subscription(
            String, 'current_state', self.state_callback, 10)

        # Now listen to the goal pose published by the state machine
        self.create_subscription(
            PoseStamped, 'goal_position', self.goal_callback, 10)

        self.path_publisher = self.create_publisher(Path, 'planned_path', 10)

        # Final fixed goal (could also be a param)
        self.final_goal = (2.0, 0.0, 0.0)

    def state_callback(self, msg: String):
        """Keep track of IDLE→SCANNING→PLANNING→EXECUTION."""
        self.current_state = msg.data

    def goal_callback(self, msg: PoseStamped):
        """
        SCANNING state stores cube pose on /goal_position,
        then state machine enters PLANNING and publishes here.
        We compute the path when in PLANNING.
        """
        if self.current_state != 'PLANNING':
            return

        # msg.pose.position is the cube’s position
        cx = msg.pose.position.x
        cy = msg.pose.position.y
        cz = msg.pose.position.z

        # final goal
        gx, gy, gz = self.final_goal

        now = self.get_clock().now().to_msg()
        path = Path()
        path.header.stamp = now
        path.header.frame_id = 'map'

        # segment 1: origin → cube
        pts1 = self.interpolate((0,0,0), (cx, cy, cz), 10)
        # segment 2: cube → final goal
        pts2 = self.interpolate((cx, cy, cz), (gx, gy, gz), 10)

        for x,y,z in pts1 + pts2:
            ps = PoseStamped()
            ps.header.stamp = now
            ps.header.frame_id = 'map'
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.position.z = z
            path.poses.append(ps)

        self.path_publisher.publish(path)
        self.get_logger().info('Published planned_path via cube→goal')

    @staticmethod
    def interpolate(start, end, num_pts):
        xs = np.linspace(start[0], end[0], num_pts)
        ys = np.linspace(start[1], end[1], num_pts)
        zs = np.linspace(start[2], end[2], num_pts)
        return list(zip(xs, ys, zs))

def main(args=None):
    rclpy.init(args=args)
    node = MotionPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
