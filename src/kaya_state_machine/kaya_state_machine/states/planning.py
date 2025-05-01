# src/kaya_state_machine/kaya_state_machine/states/planning.py

from geometry_msgs.msg import PoseStamped, PointStamped
from nav_msgs.msg import Path

class PlanningState:
    def __init__(self, machine):
        self.node = machine
        self.vector_sub = None
        self.path_sub = None

        # Publisher for goal_position (state-local)
        self.goal_pub = self.node.create_publisher(
            PoseStamped, 'goal_position', 10)

    def on_enter(self):
        self.node.get_logger().info('PLANNING: subscribing to /cube_vector')
        self.vector_sub = self.node.create_subscription(
            PointStamped,
            'cube_vector',
            self.vector_callback,
            10
        )

    def vector_callback(self, msg: PointStamped):
        self.node.get_logger().info(
            f'Received vector: x={msg.point.x}, y={msg.point.y}, z={msg.point.z}')

        # build a PoseStamped goal
        goal = PoseStamped()
        goal.header = msg.header
        goal.pose.position = msg.point

        # publish via the state-local publisher
        self.node.get_logger().info('Publishing goal_position for planner')
        self.goal_pub.publish(goal)

        # cleanup vector subscriber
        self.node.destroy_subscription(self.vector_sub)
        self.vector_sub = None

        # now subscribe to planned_path
        self.node.get_logger().info('Subscribing to /planned_path')
        self.path_sub = self.node.create_subscription(
            Path,
            'planned_path',
            self.path_callback,
            10
        )

    def path_callback(self, msg: Path):
        self.node.get_logger().info('Path received, transitioning to EXECUTION')
        # cleanup
        self.node.destroy_subscription(self.path_sub)
        self.path_sub = None
        # transition
        self.node.transition_to('EXECUTION')

    def on_exit(self):
        # ensure cleanup
        if self.vector_sub:
            self.node.destroy_subscription(self.vector_sub)
            self.vector_sub = None
        if self.path_sub:
            self.node.destroy_subscription(self.path_sub)
            self.path_sub = None
