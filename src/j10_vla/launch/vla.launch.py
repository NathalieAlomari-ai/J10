"""Start vla_inference_node with the Phase 4 scripted backend."""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory('j10_vla'), 'config', 'vla.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='Parameter file for vla_inference_node.',
        ),
        # Plain Node, not a composable one: this is the Python half of the split in
        # docs/ARCHITECTURE.md section 5. Keeping it in its own process is the point --
        # inference must never be able to stall the C++ hot path, and the intra-process
        # zero-copy that composition buys is worth nothing across a language boundary.
        Node(
            package='j10_vla',
            executable='vla_inference_node',
            name='vla_inference_node',
            output='screen',
            parameters=[LaunchConfiguration('params_file')],
        ),
    ])
