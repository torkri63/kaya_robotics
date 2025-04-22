from dynamixel_sdk import *
import time

portHandler = PortHandler("/dev/ttyUSB0")
packetHandler = PacketHandler(2.0)  # Ensure Protocol 2.0 is used

DXL_ID = 254  # Change if needed
ADDR_TORQUE_ENABLE = 64
TORQUE_ENABLE = 1
ADDR_OPERATING_MODE = 11
VELOCITY_CONTROL_MODE = 1  # Velocity control mode
ADDR_GOAL_VELOCITY = 104


portHandler.openPort()
portHandler.setBaudRate(57600)  # Change if needed

dxl_error3 = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_OPERATING_MODE, VELOCITY_CONTROL_MODE)
if dxl_error3 != 0:
    print(f"Motor {DXL_ID} has error {dxl_error3}")

dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)

if dxl_comm_result != COMM_SUCCESS:
    print("Communication Error:", dxl_comm_result)
elif dxl_error:
    print("Dynamixel Error:", dxl_error)
else:
    print("Torque enabled successfully!")


dxl_comm_result2, dxl_error2 = packetHandler.write4ByteTxRx(portHandler, DXL_ID, ADDR_GOAL_VELOCITY, 100)

if dxl_comm_result2 != COMM_SUCCESS:
    print("Communication Error:", dxl_comm_result2)
elif dxl_error2:
    print("Dynamixel Error:", dxl_error2)
else:
    print("Velocity enabled successfully!")


time.sleep(2)  # Add delay

packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, 0)
portHandler.closePort()

