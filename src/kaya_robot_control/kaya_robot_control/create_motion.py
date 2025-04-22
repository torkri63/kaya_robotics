#! /usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from dynamixel_sdk import *
import time

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
# DXL_IDS = [1]


class SetMotion(Node):
    def __init__(self):
        super().__init__("motion_dynamixel")
        self.port_handler = PortHandler(DEVICENAME)
        self.packet_handler = PacketHandler(PROTOCOL_VERSION)

        # Setting up a timer to periodically update velocities
       # self.timer = self.create_timer(5.0, self.velocity_callback)  # Call velocity_callback every 1 second

           # Subscribe to /cmd_vel topic
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.velocity_callback,
            10  # Queue size
        )
        self.get_logger().info("Subscribed to /cmd_vel")

    def init_dynamixel(self):
        """ Initialize the Dynamixel communication once """
        if self.port_handler.openPort() and self.port_handler.setBaudRate(BAUDRATE):
            self.get_logger().info("Dynamixel communication established")
        else:
            self.get_logger().error("Failed to open port or set baud rate")
            return False

        # Set motor to velocity control mode (Operating Mode = 1)
        for dxl_id in DXL_IDS:
            dxl_comm_result, dxl_error = self.packet_handler.write1ByteTxRx(self.port_handler, dxl_id, ADDR_OPERATING_MODE, VELOCITY_CONTROL_MODE)
            if dxl_comm_result != COMM_SUCCESS or dxl_error:
                self.get_logger().error(f"Error setting operating mode for motor {dxl_id} (comm_result={dxl_comm_result}, error={dxl_error})")
                return False
            self.get_logger().info(f"Operating mode set to Velocity Control for motor {dxl_id}")
            #time.sleep(5.0)
        # Enable motor torque
        for dxl_id in DXL_IDS:
            dxl_comm_result, dxl_error = self.packet_handler.write1ByteTxRx(self.port_handler, dxl_id, ADDR_TORQUE_ENABLE, 1)
            if dxl_comm_result != COMM_SUCCESS or dxl_error:
                self.get_logger().error(f"Error enabling torque for motor {dxl_id} (comm_result={dxl_comm_result}, error={dxl_error})")
                return False
            self.get_logger().info(f"Torque enabled for motor {dxl_id}")
            #time.sleep(5.0)

        return True  # Return True only if all operations succeed


    def velocity_callback(self, msg):
        
        linear_speed = msg.linear.x  # Forward speed
        #angular_speed = msg.angular.z  # Rotation speed

        # Example conversion (adjust as needed)
        left_motor_speed = int((linear_speed) * 100)
        right_motor_speed = int((linear_speed) * 100)
        motor_3_speed = int(linear_speed * 100)  # Example for third motor

        # Ensure values are within valid range (-1023 to 1023)
        left_motor_speed = max(-1023, min(1023, left_motor_speed))
        right_motor_speed = max(-1023, min(1023, right_motor_speed))
        motor_3_speed = max(-1023, min(1023, motor_3_speed))

        # Send velocity commands to motors
        for dxl_id, vel in zip(DXL_IDS, [left_motor_speed, right_motor_speed, motor_3_speed]):
            dxl_comm_result, dxl_error = self.packet_handler.write4ByteTxRx(self.port_handler, dxl_id, ADDR_GOAL_VELOCITY, vel)
            if dxl_comm_result != COMM_SUCCESS:
                self.get_logger().error(f"Failed to set velocity for motor {dxl_id}")
            elif dxl_error:
                self.get_logger().error(f"Motor {dxl_id} has error {dxl_error}")
            else:
                self.get_logger().info(f"Set velocity for motor {dxl_id}: {vel}")


    def destroy_node(self):
        # Disable torque before shutting down
        for dxl_id in DXL_IDS:
            self.packet_handler.write1ByteTxRx(self.port_handler, dxl_id, ADDR_TORQUE_ENABLE, 0)
        self.port_handler.closePort()
        super().destroy_node()



def main(args=None):

    rclpy.init(args=args)
    
    node = SetMotion() 
    
    if node.init_dynamixel():
        print("Dynamixel initialized successfully!")  
        try: 
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            rclpy.shutdown()
    else:
        print("Failed to initialize Dynamixel motors.")  


if __name__ == '__main__':
    main()
