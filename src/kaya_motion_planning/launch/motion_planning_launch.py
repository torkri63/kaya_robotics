from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='kaya_motion_planning',
            executable='motion_planner',
            name='motion_planner',
            output='screen',
            remappings=[
                # You can remap these if your topics live under a namespace
                ('robot_position', 'robot_position'),
                ('goal_position',  'goal_position'),
                ('planned_path',   'planned_path'),
            ],
        ),
    ])
