from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='imu_driver', executable='imu_node', name='imu_driver', output='screen'
        ),
        Node(
            package='motor_driver', executable='motor_controller_node', name='motor_controller', output='screen'
        ),
        Node(
            package='camera_driver', executable='camera_node', name='camera_driver', output='screen'
        ),
    ])