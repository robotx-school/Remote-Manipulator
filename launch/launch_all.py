from launch import LaunchDescription
import os
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node

def generate_launch_description():
    # config = os.path.join(
    #   "./",
    #   'config',
    #   'params.yaml'
    # )
    return LaunchDescription([
        Node(
            package="camera_node",
            namespace="/",
            executable="camera_node",
            name="camera_node",
        ),
        Node(
            package="urx_node",
            namespace="/",
            executable="urx_node",
            name="urx_node",
            
        ),
        Node(
            package="flask_node",
            namespace="/",
            executable="flask_node",
            name="flask_node",
            
        ),
        Node(
            package="socket_camera",
            namespace="/",
            executable="socket_camera",
            name="socket_camera",
            
        ),
        Node(
            package="socket_robot",
            namespace="/",
            executable="socket_robot",
            name="socket_robot",
            
        ),
        Node(
            package="robot_control",
            namespace="/",
            executable="robot_control",
            name="robot_control",
        )
    ])