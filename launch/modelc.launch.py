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
            package='ros2_whill',
            executable='ros2_whill',
            namespace='',
            output='screen',
            respawn=True,
            parameters=[
                os.path.join(pkg_dir, "params", "sample_param.yaml")
            ]
        ),
    ]
    
    return LaunchDescription(list)