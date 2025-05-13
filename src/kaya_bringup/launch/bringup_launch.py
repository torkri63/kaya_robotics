from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_bringup = get_package_share_directory('kaya_bringup')
    pkg_motion = get_package_share_directory('kaya_motion_planning')
    pkg_control = get_package_share_directory('kaya_robot_control')
    pkg_state   = get_package_share_directory('kaya_state_machine')

    return LaunchDescription([
        # hardware (stub)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_bringup, 'launch', 'hardware_launch.py')
            )
        ),
        # software (stub)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_bringup, 'launch', 'software_launch.py')
            )
        ),
        # motion planning (must come from kaya_motion_planning!)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_motion, 'launch', 'motion_planning_launch.py')
            )
        ),
        # robot control
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_control, 'launch', 'robot_control_launch.py')
            )
        ),
        # state machine
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_state, 'launch', 'state_machine_launch.py')
            )
        ),
    ])
