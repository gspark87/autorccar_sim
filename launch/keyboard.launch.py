import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share_dir = get_package_share_directory('autorccar_sim')

    return LaunchDescription([
        Node(
            package='autorccar_sim',
            executable='sim_keyboard',
            name='simulation_keyboard_publisher',
            output='screen'
        )
    ])