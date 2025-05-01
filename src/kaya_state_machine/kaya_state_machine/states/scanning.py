# src/kaya_state_machine/kaya_state_machine/states/scanning.py

from kaya_msgs.msg import CubeInfo
from geometry_msgs.msg import PoseStamped

class ScanningState:
    def __init__(self, machine):
        self.node = machine
        self.cube_sub = None

    def on_enter(self):
        self.node.get_logger().info('SCANNING: waiting for /cube_info')
        # Subscribe once; destroy when done
        self.cube_sub = self.node.create_subscription(
            CubeInfo,
            'cube_info',
            self.cube_callback,
            10)

    def cube_callback(self, msg: CubeInfo):
        if not msg.detected:
            return  # ignore if detection is false
        self.node.get_logger().info(
            f'Cube detected at x={msg.point.x}, y={msg.point.y}, z={msg.point.z}')
        # Convert to PoseStamped for the planner
        ps = PoseStamped()
        ps.header = msg.header
        ps.pose.position.x = msg.point.x
        ps.pose.position.y = msg.point.y
        ps.pose.position.z = msg.point.z
        self.node.goal_pose = ps

        # Clean up subscription to avoid repeated callbacks
        self.node.destroy_subscription(self.cube_sub)
        self.cube_sub = None

        # Transition to PLANNING
        self.node.transition_to('PLANNING')

    def on_exit(self):
        # Ensure the subscription is removed
        if self.cube_sub:
            self.node.destroy_subscription(self.cube_sub)
            self.cube_sub = None
