# src/kaya_state_machine/kaya_state_machine/states/idle.py

from std_msgs.msg import String

class IdleState:
    def __init__(self, machine):
        self.node = machine
        # subscribe to start_button topic
        self.start_sub = self.node.create_subscription(
            String, 'start_button', self.start_callback, 10)

    def on_enter(self):
        self.node.get_logger().info('IDLE: waiting for start_button (publish "PRESSED")')

    def start_callback(self, msg: String):
        if msg.data == 'PRESSED' and self.node.current_state_name == 'IDLE':
            self.node.get_logger().info('start_button PRESSED, transitioning to SCANNING')
            self.node.transition_to('SCANNING')

    def on_exit(self):
        # clean up or leave subscriber active for next time
        pass
