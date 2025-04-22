#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from dynamixel_sdk import *

# Control table addresses for XC430-150W
ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_VELOCITY = 104
ADDR_PRESENT_VELOCITY = 128
LEN_GOAL_VELOCITY = 4 # 4-byte data
ADDR_OPERATING_MODE = 11
VELOCITY_CONTROL_MODE = 1  # Velocity control mode
ADDR_ERROR = 41  

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
        
        # Set motor to velocity control mode (Operating Mode = 1)
        for dxl_id in DXL_IDS:
            self.packet_handler.write1ByteTxRx(self.port_handler, dxl_id, ADDR_OPERATING_MODE, VELOCITY_CONTROL_MODE)

            error = self.packet_handler.read1ByteTxRx(self.port_handler, dxl_id, ADDR_ERROR)
            if error != 0:
                print(f"Motor {dxl_id} has error {error}")


        # Enable motor torque
        for dxl_id in DXL_IDS:
            self.packet_handler.write1ByteTxRx(self.port_handler, dxl_id, ADDR_TORQUE_ENABLE, 1)
            response = self.packet_handler.read1ByteTxRx(self.port_handler, dxl_id, ADDR_TORQUE_ENABLE)

            # Assuming the first element of response is the result and second is the torque status
            if response == 1:
                print(f"Torque enabled for motor {dxl_id}")
            else:
                print(f"Failed to enable torque for motor {dxl_id}")
            if response[0] == COMM_SUCCESS:
                torque_enabled = response[1]
                self.get_logger().info(f"Motor {dxl_id} torque enabled: {torque_enabled}")
            else:
                self.get_logger().error(f"Failed to enable torque for motor {dxl_id} with result: {response[0]}")

        # Subscribe to velocity commands
        self.subscription = self.create_subscription(Twist, '/cmd_vel', self.velocity_callback, 10)

    def velocity_callback(self, msg):
        # Use fixed velocities for testing
       # left_motor_speed = 500  # Test with a moderate value
       # right_motor_speed = 500
       # motor_2_speed = 500
        
         # Define desired RPM values
        left_motor_rpm = 50  # Example RPM for motor 1
        right_motor_rpm = 50  # Example RPM for motor 2
        motor_2_rpm = 50  # Example RPM for motor 3

        # Convert RPM to Dynamixel velocity units (using 1/0.229)
        left_motor_speed = int(left_motor_rpm / 0.229)
        right_motor_speed = int(right_motor_rpm / 0.229)
        motor_2_speed = int(motor_2_rpm / 0.229)

        # Ensure values are within valid range (-1023 to 1023)
        left_motor_speed = max(-1023, min(1023, left_motor_speed))
        right_motor_speed = max(-1023, min(1023, right_motor_speed))
        motor_2_speed = max(-1023, min(1023, motor_2_speed))

        # Send velocity commands to motors
        velocities = {
        1: left_motor_speed,  # Left motor
        2: motor_2_speed,  # Middle motor
        3: right_motor_speed  # Right motor
        }

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
