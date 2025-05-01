from launch import LaunchDescription
from launch.actions import LogInfo

def generate_launch_description():
    return LaunchDescription([
        LogInfo(msg='⚠️  [hardware_launch] hardware nodes are stubbed out.'),
        # Later, when you add real drivers, uncomment these:
        # Node(package='imu_driver', executable='imu_node', name='imu_driver', output='screen'),
        # Node(package='motor_driver', executable='motor_controller_node', name='motor_controller', output='screen'),
        # Node(package='camera_driver', executable='camera_node', name='camera_driver', output='screen'),
    ])
