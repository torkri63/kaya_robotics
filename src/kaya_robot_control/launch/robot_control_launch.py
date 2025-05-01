from launch import LaunchDescription
from launch.actions import LogInfo

def generate_launch_description():
    return LaunchDescription([
        LogInfo(msg='⚠️  [robot_control_launch] control nodes are stubbed out until EXECUTION.'),
        # later, when in EXECUTION, launch your real Node() entries
    ])
