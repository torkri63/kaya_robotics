from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cube_detection',
            executable='live_capture_node',  
            name='live_capture_node',
            output='screen'
        ),
        Node(
            package='cube_detection',
            executable='cube_detection_node',  
            name='cube_detection_node',
            output='screen'
        ),
        Node(
            package='cube_detection',
            executable='motion_planning_subscriber',  
            name='motion_planning_subscriber',
            output='screen'
        )
    ])