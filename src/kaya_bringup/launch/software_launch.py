from launch import LaunchDescription
from launch.actions import LogInfo

def generate_launch_description():
    return LaunchDescription([
        LogInfo(msg='⚠️  [software_launch] odometry and related nodes are stubbed out.'),
        # Later, when you add real software nodes, uncomment and adjust these:
        # from launch_ros.actions import Node
        # Node(package='odometry', executable='odometry_node', name='odometry', output='screen'),
        # Node(package='some_other_pkg', executable='other_node', name='other', output='screen'),
    ])
