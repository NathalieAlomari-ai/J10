"""Start the latency monitor, and optionally the dataset recorder."""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory('j10_telemetry'), 'config', 'telemetry.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params,
                              description='Parameter file for the telemetry nodes.'),
        DeclareLaunchArgument(
            'recorder', default_value='true',
            description='Also start dataset_recorder_node. It stays idle until '
                        '/j10/telemetry/record is called, so this is cheap to leave on.'),

        Node(
            package='j10_telemetry',
            executable='latency_monitor_node',
            name='latency_monitor_node',
            output='screen',
            parameters=[LaunchConfiguration('params_file')],
        ),
        Node(
            package='j10_telemetry',
            executable='dataset_recorder_node',
            name='dataset_recorder_node',
            output='screen',
            parameters=[LaunchConfiguration('params_file')],
            condition=IfCondition(LaunchConfiguration('recorder')),
        ),
    ])
