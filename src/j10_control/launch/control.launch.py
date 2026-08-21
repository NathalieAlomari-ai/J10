"""Bring up the motion controller.

Sits on the realtime path, so in flight it belongs in the same ``component_container_mt``
as the video receiver, safety filter and MAVLink bridge -- ``use_composition:=true`` loads
it into an existing container instead of starting its own process.
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
                [FindPackageShare('j10_control'), 'config', 'control.yaml']
            ),
            description='Motion controller parameter file.',
        ),
        DeclareLaunchArgument(
            'use_composition',
            default_value='false',
            description='Load into an existing component container.',
        ),
        DeclareLaunchArgument(
            'container',
            default_value='/j10_mavlink_container',
            description='Target container when use_composition is true.',
        ),
    ]

    standalone = Node(
        package='j10_control',
        executable='motion_controller_node',
        name='motion_controller_node',
        parameters=[params_file],
        output='screen',
        condition=UnlessCondition(use_composition),
    )

    composed = LoadComposableNodes(
        target_container=container,
        composable_node_descriptions=[
            ComposableNode(
                package='j10_control',
                plugin='j10_control::MotionControllerNode',
                name='motion_controller_node',
                parameters=[params_file],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
        ],
        condition=IfCondition(use_composition),
    )

    return LaunchDescription(args + [standalone, composed])
