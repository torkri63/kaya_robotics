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
            output='screen',
            parameters=[
                {'input_topic': '/cube/position'},
                {'output_position_topic': '/motion_planning/cube_position'},
                {'output_distance_topic': '/cube/distance'},
                {'source_frame': 'camera_frame'},
                {'target_frame': 'robot_base_frame'}
            ]
        )
    ])
