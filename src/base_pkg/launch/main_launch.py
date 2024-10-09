from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='base_pkg',
            executable='keyboard_publisher',
            name='keyboard_publisher',
            output='screen',
        ),
    ])