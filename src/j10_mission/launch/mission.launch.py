"""Start mission_manager_node."""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory('j10_mission'), 'config', 'mission.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='Parameter file for mission_manager_node.',
        ),
        Node(
            package='j10_mission',
            executable='mission_manager_node',
            name='mission_manager_node',
            output='screen',
            parameters=[LaunchConfiguration('params_file')],
        ),
    ])
