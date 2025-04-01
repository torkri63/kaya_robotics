import time

class PlanningState:
    def __init__(self, machine):
        self.machine = machine
        self.node = machine.node
        self.start_time = time.time()
        self.mock_cube_visible = True
        self.mock_vector_to_cube = [1.0, 0.5]  # Placeholder
        self.mock_endpoint = [2.0, 0.0]
        self.planned = False

    def on_enter(self):
        self.node.get_logger().info("Entered PLANNING state")
        self.node.get_logger().info("Waiting for vector info from camera...")

    def on_update(self):
        current_time = time.time()

        # Simulate receiving message from camera after 3 seconds
        if not self.planned and current_time - self.start_time > 3.0:
            if self.mock_cube_visible:
                self.node.get_logger().info(f"Cube visible. Vector: {self.mock_vector_to_cube}")
                self.node.get_logger().info(f"Calculating trajectory to endpoint {self.mock_endpoint}")
                self.node.get_logger().info("Trajectory planning successful. Switching to EXECUTION.")
                self.planned = True
                from kaya_state_machine.states.execution import ExecutionState
                self.machine.transition_to(ExecutionState)
            else:
                self.node.get_logger().info("Cube lost. Returning to SCANNING.")
                from kaya_state_machine.states.scanning import ScanningState
                self.machine.transition_to(ScanningState)

    def on_exit(self):
        self.node.get_logger().info("Exiting PLANNING state")
