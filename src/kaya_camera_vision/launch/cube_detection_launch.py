#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        # 1) Publish raw RealSense frames
        Node(
            package='kaya_camera_vision',
            executable='live_capture_node',
            name='live_capture_node',
            output='screen',
        ),

        # 2) Run YOLO + depth → PoseArray on /cube/positions
        Node(
            package='kaya_camera_vision',
            executable='cube_detection_node',
            name='cube_detection_node',
            output='screen',
        ),

        # 3) Flatten PoseArray → /cube_info
        Node(
            package='kaya_camera_vision',
            executable='motion_planning_subscriber',
            name='motion_planning_subscriber',
            output='screen',
            parameters=[{
                'input_topic': '/cube/positions'
            }]
        ),

    ])
