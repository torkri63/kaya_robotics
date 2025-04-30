class IdleState:
    def __init__(self, machine):
        self.machine = machine
        self.node = machine.node

    def on_enter(self):
        self.node.get_logger().info("Entered IDLE state")
        self.node.get_logger().info("Press ENTER to start scanning...")

    def on_update(self):
        import sys
        import select

        if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
            line = sys.stdin.readline()
            if line.strip() == "":
                self.node.get_logger().info("ENTER pressed. Switching to SCANNING.")
                from kaya_state_machine.states.scanning import ScanningState
                self.machine.transition_to(ScanningState)

    def on_exit(self):
        self.node.get_logger().info("Exiting IDLE state")
