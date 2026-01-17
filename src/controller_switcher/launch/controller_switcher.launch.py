from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="controller_switcher",
            executable="controller_switcher",
            output="screen"
        )
    ])
