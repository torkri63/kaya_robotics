import rclpy
from rclpy.node import Node
from std_msgs.msg import String                          # ⬅️ import for state topic

from kaya_state_machine.states.execution import ExecutionState
from kaya_state_machine.states.idle import IdleState
from kaya_state_machine.states.planning import PlanningState
from kaya_state_machine.states.scanning import ScanningState

class StateMachineNode(Node):
    def __init__(self):
        super().__init__('state_machine')
        self.node = self

        # State machine internals
        self.states = {
            'IDLE': IdleState(self),
            'SCANNING': ScanningState(self),
            'EXECUTION': ExecutionState(self),
            'PLANNING': PlanningState(self),
        }
        self.current_state_name = 'IDLE'
        self.state = self.states[self.current_state_name]

        # Publisher for state transitions
        self.state_pub = self.create_publisher(String, 'current_state', 10)   # ⬅️

        # Subscriber for button (could be topic or input)
        self.create_subscription(
            String,
            'start_button',
            self.start_button_callback,
            10
        )

        self.get_logger().info(f'State machine started in {self.current_state_name}')

        # Publish initial state
        self.publish_state()                                               # ⬅️

    def start_button_callback(self, msg):
        # When the start button is pressed, transition out of IDLE
        if self.current_state_name == 'IDLE' and msg.data == 'PRESSED':
            self.transition_to('SCANNING')

    def transition_to(self, new_state_name):
        # run exit of old state
        self.state.on_exit()

        # switch
        self.current_state_name = new_state_name
        self.state = self.states[new_state_name]

        # publish the updated state
        self.publish_state()                                               # ⬅️

        # run enter of new state
        self.state.on_enter()

    def publish_state(self):
        """Publish the current state name on /current_state."""
        msg = String()
        msg.data = self.current_state_name
        self.state_pub.publish(msg)                                       # ⬅️
        self.get_logger().info(f'Published state: {msg.data}')

    def run(self):
        rclpy.spin(self)

def main(args=None):
    rclpy.init(args=args)
    node = StateMachineNode()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
