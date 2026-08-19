"""Bring up the safety filter.

Standalone by default. The filter sits on the realtime path, so in flight it belongs in
the same ``component_container_mt`` as the video receiver, motion controller and MAVLink
bridge (see docs/ARCHITECTURE.md section 4) -- ``use_composition:=true`` loads it into an
existing container instead.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = LaunchConfiguration('params_file')
    use_composition = LaunchConfiguration('use_composition')
    container = LaunchConfiguration('container')

    args = [
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution(
                [FindPackageShare('j10_safety'), 'config', 'safety.yaml']
            ),
            description='Safety envelope parameter file.',
        ),
        DeclareLaunchArgument(
            'use_composition',
            default_value='false',
            description='Load into an existing component container instead of running '
                        'as its own process.',
        ),
        DeclareLaunchArgument(
            'container',
            default_value='/j10_mavlink_container',
            description='Target container when use_composition is true.',
        ),
    ]

    standalone = Node(
        package='j10_safety',
        executable='safety_filter_node',
        name='safety_filter_node',
        parameters=[params_file],
        output='screen',
        condition=UnlessCondition(use_composition),
    )

    composed = LoadComposableNodes(
        target_container=container,
        composable_node_descriptions=[
            ComposableNode(
                package='j10_safety',
                plugin='j10_safety::SafetyFilterNode',
                name='safety_filter_node',
                parameters=[params_file],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
        ],
        condition=IfCondition(use_composition),
    )

    return LaunchDescription(args + [standalone, composed])
