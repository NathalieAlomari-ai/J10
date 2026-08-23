"""Start teleop_override_node, and optionally the joy driver that feeds it."""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory('j10_teleop'), 'config', 'teleop.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='Parameter file for teleop_override_node.',
        ),
        DeclareLaunchArgument(
            'joy',
            default_value='true',
            description='Also start the joy_node driver that publishes /joy.',
        ),
        DeclareLaunchArgument(
            'joy_device_id',
            default_value='0',
            description='Which /dev/input/js* the joy driver opens.',
        ),

        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[{
                'device_id': LaunchConfiguration('joy_device_id'),
                # Republish periodically rather than only on change. The override node
                # treats silence as a released deadman, so a controller held perfectly
                # still must keep saying so.
                'autorepeat_rate': 20.0,
            }],
            condition=IfCondition(LaunchConfiguration('joy')),
        ),

        Node(
            package='j10_teleop',
            executable='teleop_override_node',
            name='teleop_override_node',
            output='screen',
            parameters=[LaunchConfiguration('params_file')],
        ),
    ])
