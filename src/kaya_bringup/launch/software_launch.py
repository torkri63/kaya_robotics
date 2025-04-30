from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='odometry', executable='odom_node', name='odometry', output='screen'),
        Node(package='nav2_amcl', executable='amcl', name='amcl', output='screen'),
        Node(package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator', output='screen'),
        Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager',
            name='lifecycle_manager_navigation', output='screen',
            parameters=[{'use_sim_time': False}, {'autostart': True}, {'node_names': ['bt_navigator', 'amcl']}]
        ),
        Node(package='motionplanning', executable='planner_node', name='motion_planner', output='screen'),
        Node(package='control', executable='controller_node', name='motion_controller', output='screen'),
        Node(package='vision', executable='vision_node', name='vision_processor', output='screen'),
        Node(package='state_machine', executable='state_machine_node', name='state_machine', output='screen'),
    ])