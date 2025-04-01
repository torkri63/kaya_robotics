import rclpy
from rclpy.node import Node

from kaya_state_machine.states.idle import IdleState
from kaya_state_machine.states.scanning import ScanningState
from kaya_state_machine.states.planning import PlanningState
from kaya_state_machine.states.execution import ExecutionState

class StateMachine:
    def __init__(self, node):
        self.node = node
        self.state = IdleState(self)  # Start in Idle
        self.state.on_enter()  # 👈 Call it right after creating the first state


    def transition_to(self, new_state_class):
        self.node.get_logger().info(f"Transitioning to {new_state_class.__name__}")
        self.state.on_exit()
        self.state = new_state_class(self)
        self.state.on_enter()

    def update(self):
        self.state.on_update()

class MainNode(Node):
    def __init__(self):
        super().__init__('kaya_state_machine')
        self.state_machine = StateMachine(self)
        self.get_logger().info("State machine started in IDLE")

        self.timer = self.create_timer(0.1, self.state_machine.update)  # 10Hz loop

def main(args=None):
    import rclpy
    rclpy.init(args=args)
    node = MainNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

