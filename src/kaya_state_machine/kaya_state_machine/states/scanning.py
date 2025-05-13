from kaya_msgs.msg import CubeInfo
from geometry_msgs.msg import PoseStamped

class ScanningState:
    def __init__(self, machine):
        # machine is your StateMachineNode instance
        self.node = machine
        self.cube_sub = None

    def on_enter(self):
        self.node.get_logger().info('SCANNING: waiting for /cube_info')
        # Subscribe once; we'll destroy it after the first positive detection
        self.cube_sub = self.node.create_subscription(
            CubeInfo,
            'cube_info',
            self.cube_callback,
            10
        )

    def cube_callback(self, msg: CubeInfo):
        # Only act on a positive detection
        if not msg.detected:
            return

        x = msg.point.x
        y = msg.point.y
        z = msg.point.z
        self.node.get_logger().info(
            f'Cube detected at x={x:.2f}, y={y:.2f}, z={z:.2f}')

        # Pack into a PoseStamped for the planner
        ps = PoseStamped()
        ps.header = msg.header
        ps.pose.position.x = x
        ps.pose.position.y = y
        ps.pose.position.z = z

        # Store on the machine for the next state
        self.node.goal_pose = ps

        # Clean up this subscription so it only fires once
        self.node.destroy_subscription(self.cube_sub)
        self.cube_sub = None

        # Transition to PLANNING
        self.node.transition_to('PLANNING')

    def on_exit(self):
        # Just in case on_enter ran twice without cleanup
        if self.cube_sub:
            self.node.destroy_subscription(self.cube_sub)
            self.cube_sub = None
