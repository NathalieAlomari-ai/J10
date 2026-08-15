"""Phase 1 bring-up: Gazebo + ArduPilot SITL + MAVROS + the j10_mavlink nodes.

Each stage can be switched off so the pieces can be run in separate terminals while
debugging, which is usually what you want the first time through:

    ros2 launch j10_sim sitl.launch.py sitl:=false        # run run_sitl.sh yourself
    ros2 launch j10_sim sitl.launch.py gazebo:=false      # attach to an existing Gazebo
    ros2 launch j10_sim sitl.launch.py mavros:=false j10:=false

Phase 1 exit criterion (docs/ARCHITECTURE.md section 8): a TwistStamped published on
/j10/cmd_vel_safe moves the vehicle in the commanded body-frame direction, and releasing
the command returns it to hover within 1 s. See src/j10_sim/README.md for the runbook.
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    sim_share = get_package_share_directory('j10_sim')

    gazebo = LaunchConfiguration('gazebo')
    sitl = LaunchConfiguration('sitl')
    mavros = LaunchConfiguration('mavros')
    j10 = LaunchConfiguration('j10')

    default_ardupilot_dir = os.environ.get(
        'ARDUPILOT_DIR', os.path.join(os.path.expanduser('~'), 'ardupilot')
    )
    default_ardupilot_gazebo = os.environ.get(
        'ARDUPILOT_GAZEBO', os.path.join(os.path.expanduser('~'), 'ardupilot_gazebo')
    )

    args = [
        DeclareLaunchArgument('gazebo', default_value='true',
                              description='Start Gazebo on the indoor world.'),
        DeclareLaunchArgument('sitl', default_value='true',
                              description='Start ArduCopter SITL via run_sitl.sh.'),
        DeclareLaunchArgument('mavros', default_value='true',
                              description='Start mavros_node.'),
        DeclareLaunchArgument('j10', default_value='true',
                              description='Start mavlink_bridge_node and '
                                          'vehicle_state_node.'),

        DeclareLaunchArgument('headless', default_value='false',
                              description='Run Gazebo without a GUI.'),
        DeclareLaunchArgument('world', default_value='',
                              description='SDF world override; empty means j10_indoor.sdf.'),

        DeclareLaunchArgument('ardupilot_dir', default_value=default_ardupilot_dir,
                              description='ArduPilot source tree containing '
                                          'Tools/autotest/sim_vehicle.py.'),
        DeclareLaunchArgument('ardupilot_gazebo', default_value=default_ardupilot_gazebo,
                              description='Built ardupilot_gazebo checkout.'),

        DeclareLaunchArgument('fcu_url', default_value='udp://:14551@',
                              description='MAVROS link to SITL. Must match the --out '
                                          'endpoint run_sitl.sh passes to sim_vehicle.py.'),
        DeclareLaunchArgument('mavros_config',
                              default_value=os.path.join(
                                  sim_share, 'config', 'mavros_apm.yaml'),
                              description='MAVROS parameter file.'),
        DeclareLaunchArgument('j10_params',
                              default_value=PathJoinSubstitution(
                                  [FindPackageShare('j10_mavlink'),
                                   'config', 'mavlink.yaml']),
                              description='Parameter file for the j10_mavlink nodes.'),

        DeclareLaunchArgument('sitl_startup_delay', default_value='5.0',
                              description='Seconds to wait after starting Gazebo before '
                                          'launching SITL. ArduPilot exits if the '
                                          "ArduPilotPlugin's JSON socket is not up yet."),
        DeclareLaunchArgument('mavros_startup_delay', default_value='10.0',
                              description='Seconds to wait before starting MAVROS, so its '
                                          'first connection attempt finds a live link.'),
    ]

    # --- Gazebo ---------------------------------------------------------------------------
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sim_share, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'headless': LaunchConfiguration('headless'),
            'ardupilot_gazebo': LaunchConfiguration('ardupilot_gazebo'),
        }.items(),
        condition=IfCondition(gazebo),
    )

    # --- ArduPilot SITL -------------------------------------------------------------------
    # Started after Gazebo: ArduPilot's JSON backend connects out to the ArduPilotPlugin and
    # gives up if nothing is listening.
    sitl_process = TimerAction(
        period=LaunchConfiguration('sitl_startup_delay'),
        actions=[
            ExecuteProcess(
                cmd=[
                    os.path.normpath(
                        os.path.join(
                            sim_share, '..', '..', 'lib', 'j10_sim', 'run_sitl.sh')),
                    '--ardupilot-dir', LaunchConfiguration('ardupilot_dir'),
                    '--param-file', os.path.join(
                        sim_share, 'config', 'sitl_indoor.parm'),
                ],
                output='screen',
                shell=False,
            )
        ],
        condition=IfCondition(sitl),
    )

    # --- MAVROS ---------------------------------------------------------------------------
    mavros_node = TimerAction(
        period=LaunchConfiguration('mavros_startup_delay'),
        actions=[
            Node(
                package='mavros',
                executable='mavros_node',
                name='mavros',
                namespace='mavros',
                output='screen',
                parameters=[
                    LaunchConfiguration('mavros_config'),
                    {'fcu_url': LaunchConfiguration('fcu_url')},
                ],
            )
        ],
        condition=IfCondition(mavros),
    )

    # --- J10 nodes ------------------------------------------------------------------------
    j10_nodes = TimerAction(
        period=LaunchConfiguration('mavros_startup_delay'),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([
                        FindPackageShare('j10_mavlink'), 'launch', 'mavlink.launch.py'
                    ])
                ),
                launch_arguments={
                    'params_file': LaunchConfiguration('j10_params'),
                }.items(),
            )
        ],
        condition=IfCondition(j10),
    )

    return LaunchDescription(args + [gazebo_launch, sitl_process, mavros_node, j10_nodes])
