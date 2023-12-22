import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    share_dir = get_package_share_directory('autorccar_sim')
    parameter_file = LaunchConfiguration('params_file')

    params_declare = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(
            share_dir, 'launch', 'config.yaml'),
        description='Path to the ROS2 parameters file to use.')

    return LaunchDescription([
        params_declare,
        Node(
            package='autorccar_sim',
            executable='sim_sensors',
            name='simulation_sensors_publisher',
            parameters=[parameter_file],
            output='screen'
        )
    ])