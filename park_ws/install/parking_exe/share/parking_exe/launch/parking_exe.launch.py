from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='parking_exe',
            executable='parking_exe_node',
            name='parking_exe_node',
            output='screen'
        ),
    ])