import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from dynamixel_sdk import *

# Control table addresses for XC430-150W
ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_VELOCITY = 104
ADDR_PRESENT_VELOCITY = 128
LEN_GOAL_VELOCITY = 4  # 4-byte data

# Protocol version and device settings
PROTOCOL_VERSION = 2.0
DEVICENAME = '/dev/ttyUSB0'  # Change based on your setup
BAUDRATE = 57600

# Motor IDs
DXL_IDS = [1, 2, 3]  # Motors ID 1, 2, and 3

class DynamixelVelocityController(Node):
    def __init__(self):
        super().__init__('dynamixel_velocity_controller')

        # Initialize Dynamixel communication
        self.port_handler = PortHandler(DEVICENAME)
        self.packet_handler = PacketHandler(PROTOCOL_VERSION)

        if self.port_handler.openPort() and self.port_handler.setBaudRate(BAUDRATE):
            self.get_logger().info("Dynamixel communication established")
        else:
            self.get_logger().error("Failed to open port or set baud rate")
            return

        # Enable motor torque
        for dxl_id in DXL_IDS:
            self.packet_handler.write1ByteTxRx(self.port_handler, dxl_id, ADDR_TORQUE_ENABLE, 1)

        # Subscribe to velocity commands
        self.subscription = self.create_subscription(Twist, '/cmd_vel', self.velocity_callback, 10)

    def velocity_callback(self, msg):
        # Convert linear velocity to Dynamixel speed value
        velocity = int(msg.linear.x * 1023)  # Scale linear velocity
        angular_velocity = int(msg.angular.z * 1023)  # Scale rotation

        # Left and right motors should have the same velocity
        left_motor_speed = velocity - angular_velocity  # Adjust for rotation
        right_motor_speed = velocity + angular_velocity  # Adjust for rotation

        # Motor 2 can be independent (modify as needed)
        motor_2_speed = velocity  # Keeping it same as linear for now

        # Ensure values are within valid range (-1023 to 1023)
        left_motor_speed = max(-1023, min(1023, left_motor_speed))
        right_motor_speed = max(-1023, min(1023, right_motor_speed))
        motor_2_speed = max(-1023, min(1023, motor_2_speed))

        # Set velocity for each motor
        velocities = {
            1: left_motor_speed,  # Left motor
            2: motor_2_speed,  # Middle motor
            3: right_motor_speed  # Right motor
        }

        # Send velocity commands to motors
        for dxl_id, vel in velocities.items():
            self.packet_handler.write4ByteTxRx(self.port_handler, dxl_id, ADDR_GOAL_VELOCITY, vel)

        self.get_logger().info(f'Set velocities: {velocities}')

    def destroy_node(self):
        # Disable torque before shutting down
        for dxl_id in DXL_IDS:
            self.packet_handler.write1ByteTxRx(self.port_handler, dxl_id, ADDR_TORQUE_ENABLE, 0)
        self.port_handler.closePort()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = DynamixelVelocityController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
