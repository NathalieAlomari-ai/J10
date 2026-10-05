"""
Bring up the autonomy stack: VLA -> control -> safety, plus the mission manager.

Deliberately does NOT start Gazebo, SITL, MAVROS or j10_mavlink. Those are the vehicle
side and belong to j10_sim's sitl.launch.py, which is usually already running (and during
bring-up is usually running in its own terminals so its logs stay readable). This file is
the layer that sits on top of a working vehicle link.

The four nodes here are the Phase 4 chain from docs/ARCHITECTURE.md section 8:

    vla_inference (5-10 Hz)  ->  /j10/vla/intent
    motion_controller (30 Hz) ->  /j10/cmd_vel_raw
    safety_filter (30 Hz)     ->  /j10/cmd_vel_safe   -> mavlink_bridge, already running
    mission_manager (5 Hz)    ->  /j10/mission/autonomy_enabled, gating the above

Nothing here can move the vehicle on its own. safety_filter starts with autonomy disabled
and mission_manager starts in IDLE with no instruction, so bringing this up next to a
flying vehicle changes nothing until a state transition is explicitly requested.
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    def params(package, filename):
        return os.path.join(get_package_share_directory(package), 'config', filename)

    args = [
        DeclareLaunchArgument('safety', default_value='true',
                              description='Start safety_filter_node.'),
        DeclareLaunchArgument('control', default_value='true',
                              description='Start motion_controller_node.'),
        DeclareLaunchArgument('vla', default_value='true',
                              description='Start vla_inference_node.'),
        DeclareLaunchArgument('mission', default_value='true',
                              description='Start mission_manager_node.'),

        DeclareLaunchArgument('safety_params', default_value=params('j10_safety', 'safety.yaml'),
                              description='Parameter file for safety_filter_node.'),
        DeclareLaunchArgument('control_params',
                              default_value=params('j10_control', 'control.yaml'),
                              description='Parameter file for motion_controller_node.'),
        DeclareLaunchArgument('vla_params', default_value=params('j10_vla', 'vla.yaml'),
                              description='Parameter file for vla_inference_node.'),
        DeclareLaunchArgument('mission_params',
                              default_value=params('j10_mission', 'mission.yaml'),
                              description='Parameter file for mission_manager_node.'),
    ]

    # Plain Nodes rather than a shared container. Composition buys intra-process zero-copy
    # between the C++ nodes, which is worth having in flight -- but during bring-up one
    # crashed node taking the other three down with it costs more than the microseconds it
    # saves, and j10_vla is Python and cannot join the container anyway. Compose the
    # C++ trio once the chain is proven, not while proving it.
    nodes = [
        Node(package='j10_safety', executable='safety_filter_node',
             name='safety_filter_node', output='screen',
             parameters=[LaunchConfiguration('safety_params')],
             condition=IfCondition(LaunchConfiguration('safety'))),

        Node(package='j10_control', executable='motion_controller_node',
             name='motion_controller_node', output='screen',
             parameters=[LaunchConfiguration('control_params')],
             condition=IfCondition(LaunchConfiguration('control'))),

        Node(package='j10_vla', executable='vla_inference_node',
             name='vla_inference_node', output='screen',
             parameters=[LaunchConfiguration('vla_params')],
             condition=IfCondition(LaunchConfiguration('vla'))),

        Node(package='j10_mission', executable='mission_manager_node',
             name='mission_manager_node', output='screen',
             parameters=[LaunchConfiguration('mission_params')],
             condition=IfCondition(LaunchConfiguration('mission'))),
    ]

    return LaunchDescription(args + nodes)
