"""Bring up the two j10_mavlink nodes.

Both nodes run inside one ``component_container_mt`` with intra-process comms, per
docs/ARCHITECTURE.md section 4. The ``_mt`` container is required, not preferred:
``mavlink_bridge_node`` blocks on MAVROS service calls from inside its own service
callbacks, which deadlocks on a single-threaded executor.

This file is included by ``j10_sim`` for SITL and will be included by ``j10_bringup`` for
real flight, so the container definition lives with the package that owns the nodes rather
than being duplicated per consumer.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_params = PathJoinSubstitution(
        [FindPackageShare('j10_mavlink'), 'config', 'mavlink.yaml']
    )

    params_file = LaunchConfiguration('params_file')
    use_composition = LaunchConfiguration('use_composition')
    log_level = LaunchConfiguration('log_level')

    args = [
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='YAML parameter file for both j10_mavlink nodes.',
        ),
        DeclareLaunchArgument(
            'use_composition',
            default_value='true',
            description=(
                'Run both nodes in one component_container_mt with intra-process comms. '
                'Set false to run them as separate processes, which is easier to debug.'
            ),
        ),
        DeclareLaunchArgument(
            'log_level',
            default_value='info',
            description='ROS logging level for the nodes.',
        ),
    ]

    container = ComposableNodeContainer(
        name='j10_mavlink_container',
        namespace='',
        package='rclcpp_components',
        # _mt is mandatory — see the module docstring.
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='j10_mavlink',
                plugin='j10_mavlink::VehicleStateNode',
                name='vehicle_state_node',
                parameters=[params_file],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
            ComposableNode(
                package='j10_mavlink',
                plugin='j10_mavlink::MavlinkBridgeNode',
                name='mavlink_bridge_node',
                parameters=[params_file],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
        ],
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
        condition=IfCondition(use_composition),
    )

    separate_nodes = [
        Node(
            package='j10_mavlink',
            executable='vehicle_state_node',
            name='vehicle_state_node',
            parameters=[params_file],
            output='screen',
            arguments=['--ros-args', '--log-level', log_level],
            condition=UnlessCondition(use_composition),
        ),
        Node(
            package='j10_mavlink',
            executable='mavlink_bridge_node',
            name='mavlink_bridge_node',
            parameters=[params_file],
            output='screen',
            arguments=['--ros-args', '--log-level', log_level],
            condition=UnlessCondition(use_composition),
        ),
    ]

    return LaunchDescription(args + [container] + separate_nodes)
