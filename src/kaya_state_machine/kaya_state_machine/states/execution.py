import time

class ExecutionState:
    def __init__(self, machine):
        self.machine = machine
        self.node = machine.node
        self.start_time = time.time()
        self.last_update_time = time.time()

    def on_enter(self):
        self.node.get_logger().info("Entered EXECUTION state")
        self.node.get_logger().info("Starting to follow trajectory...")

    def on_update(self):
        current_time = time.time()

        # Log every second
        if current_time - self.last_update_time >= 1.0:
            self.node.get_logger().info("Executing... Updating trajectory.")
            self.last_update_time = current_time

        # After 10 seconds, pretend we're done
        if current_time - self.start_time >= 10.0:
            self.node.get_logger().info("Destination reached. Returning to IDLE.")
            from kaya_state_machine.states.idle import IdleState
            self.machine.transition_to(IdleState)

    def on_exit(self):
        self.node.get_logger().info("Exiting EXECUTION state")
