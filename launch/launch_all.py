from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="camera_node",
            namespace="/",
            executable="camera_node",
            name="camera_node",
        ),
        Node(
            package="flask_node",
            namespace="/",
            executable="flask_node",
            name="flask_node",
        )
    ])