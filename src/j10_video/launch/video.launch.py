"""Start video_receiver_node."""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory('j10_video'), 'config', 'video.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params,
                              description='Parameter file for video_receiver_node.'),

        Node(
            package='j10_video',
            executable='video_receiver_node',
            name='video_receiver_node',
            output='screen',
            parameters=[LaunchConfiguration('params_file')],
        ),
    ])
