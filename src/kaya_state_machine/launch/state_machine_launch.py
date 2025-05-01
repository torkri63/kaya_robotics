from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='kaya_state_machine',
            executable='main_node',
            name='state_machine',
            output='screen',
        ),
    ])
