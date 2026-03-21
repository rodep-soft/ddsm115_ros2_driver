import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory("ddsm115_ros2_driver"),
        "config",
        "ddsm115driver_node.yaml",
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file",
            default_value=default_params,
            description="Path to the DDSM115 driver parameter file",
        ),
        Node(
            package="ddsm115_ros2_driver",
            executable="ddsm115_driver_node",
            name="ddsm115driver_node",
            output="screen",
            parameters=[LaunchConfiguration("params_file")],
        ),
    ])
