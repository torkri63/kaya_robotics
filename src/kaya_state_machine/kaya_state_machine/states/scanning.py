import time



class ScanningState:
    def __init__(self, machine):
        self.machine = machine
        self.node = machine.node
        self.last_log_time = time.time()
        self.detected = False
        self.start_time = time.time()



    def on_enter(self):
        self.node.get_logger().info("Entered SCANNING state")

    def on_update(self):
        current_time = time.time()
        if current_time - self.last_log_time >= 5.0:
            self.node.get_logger().info("Scanning...")
            self.last_log_time = current_time
        if not self.detected and current_time - self.start_time >= 10.0:
            self.node.get_logger().info("Cube detected! Switching to PLANNING.")
            self.detected = True
            from kaya_state_machine.states.planning import PlanningState
            self.machine.transition_to(PlanningState)

    def on_exit(self):
        self.node.get_logger().info("Exiting SCANNING state")
