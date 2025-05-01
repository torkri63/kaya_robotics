# src/kaya_state_machine/kaya_state_machine/states/planning.py

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

class PlanningState:
    def __init__(self, machine):
        self.node = machine
        self.path_sub = None

        # Create a state-local publisher for sending the goal to the planner
        self.goal_pub = self.node.create_publisher(
            PoseStamped, 'goal_position', 10)

    def on_enter(self):
        self.node.get_logger().info('PLANNING: publishing goal_position to planner')
        # Publish the stored goal_pose from the SCANNING state
        goal = self.node.goal_pose
        self.goal_pub.publish(goal)

        # Now subscribe to the real planner’s output
        self.node.get_logger().info('PLANNING: subscribing to /planned_path')
        self.path_sub = self.node.create_subscription(
            Path,
            'planned_path',
            self.path_callback,
            10
        )

    def path_callback(self, msg: Path):
        self.node.get_logger().info(
            f'Path received ({len(msg.poses)} poses), transitioning to EXECUTION'
        )
        # Clean up the subscription
        self.node.destroy_subscription(self.path_sub)
        self.path_sub = None
        # Transition into EXECUTION
        self.node.transition_to('EXECUTION')

    def on_exit(self):
        # Ensure any lingering subscription is removed
        if self.path_sub:
            self.node.destroy_subscription(self.path_sub)
            self.path_sub = None
