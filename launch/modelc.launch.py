import os

from launch import LaunchDescription
import launch_ros.actions
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import Node


from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory("ros2_whill")
    list = [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            namespace="",
            # parameters=[{"robot_description" : os.path.join(pkg_dir, "urdf", "modelc.urdf")}],
            remappings=[("/joint_states", "/whill/states/joint_state")],
            arguments=[os.path.join(pkg_dir, "urdf", "modelc.urdf")]
        ),
        Node(
            package="ros2_whill",
            executable="ros2_whill",
            namespace="",
            output="screen",
            respawn=True,
            parameters=[os.path.join(pkg_dir, "params", "sample_param.yaml")],
        ),
    ]

    return LaunchDescription(list)
