"""Start Gazebo on the J10 indoor world.

Deliberately shells out to ``gz sim`` rather than going through ``ros_gz_sim``. Phase 1
needs no ROS<->Gazebo bridging at all — ArduPilot talks to the ArduPilotPlugin over its own
JSON socket, and the vehicle state we care about arrives via MAVROS. The camera bridge
lands in Phase 3, which is when a ``ros_gz`` dependency starts paying for itself.
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def _is_true(value):
    return str(value).strip().lower() in ('true', '1', 'yes', 'on')


def _launch_gazebo(context, *args, **kwargs):
    """Resolve the arguments and build the single gz command they imply."""
    pkg_share = get_package_share_directory('j10_sim')

    world = LaunchConfiguration('world').perform(context)
    headless = _is_true(LaunchConfiguration('headless').perform(context))
    paused = _is_true(LaunchConfiguration('paused').perform(context))
    verbosity = LaunchConfiguration('verbosity').perform(context)
    ardupilot_gazebo = LaunchConfiguration('ardupilot_gazebo').perform(context)

    if not world:
        world = os.path.join(pkg_share, 'worlds', 'j10_indoor.sdf')

    # The iris model and the ArduPilotPlugin live in the ardupilot_gazebo checkout, not in
    # this package, so both search paths must be extended before gz sim starts.
    resource_paths = [
        os.path.join(ardupilot_gazebo, 'models'),
        os.path.join(ardupilot_gazebo, 'worlds'),
        os.path.join(pkg_share, 'worlds'),
    ]
    plugin_paths = [os.path.join(ardupilot_gazebo, 'build')]

    env = dict(os.environ)
    env['GZ_SIM_RESOURCE_PATH'] = os.pathsep.join(
        [p for p in resource_paths + [env.get('GZ_SIM_RESOURCE_PATH', '')] if p]
    )
    env['GZ_SIM_SYSTEM_PLUGIN_PATH'] = os.pathsep.join(
        [p for p in plugin_paths + [env.get('GZ_SIM_SYSTEM_PLUGIN_PATH', '')] if p]
    )

    if not os.path.isdir(ardupilot_gazebo):
        print(
            '[j10_sim] WARNING: ardupilot_gazebo not found at "{}". '
            'model://iris_with_ardupilot will not resolve and the world will load empty. '
            'Clone and build it (see src/j10_sim/README.md), then pass '
            'ardupilot_gazebo:=/path/to/ardupilot_gazebo.'.format(ardupilot_gazebo)
        )

    cmd = ['gz', 'sim', '-v', verbosity]
    if headless:
        cmd.append('-s')          # server only, no GUI
    if not paused:
        cmd.append('-r')          # run immediately instead of waiting for play
    cmd.append(world)

    return [
        ExecuteProcess(
            cmd=cmd,
            output='screen',
            additional_env={
                'GZ_SIM_RESOURCE_PATH': env['GZ_SIM_RESOURCE_PATH'],
                'GZ_SIM_SYSTEM_PLUGIN_PATH': env['GZ_SIM_SYSTEM_PLUGIN_PATH'],
            },
        )
    ]


def generate_launch_description():
    default_ardupilot_gazebo = os.environ.get(
        'ARDUPILOT_GAZEBO', os.path.join(os.path.expanduser('~'), 'ardupilot_gazebo')
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value='',
            description='SDF world file. Empty means the installed j10_indoor.sdf.',
        ),
        DeclareLaunchArgument(
            'headless',
            default_value='false',
            description='Run the server only, with no GUI.',
        ),
        DeclareLaunchArgument(
            'paused',
            default_value='false',
            description='Start the simulation paused instead of running.',
        ),
        DeclareLaunchArgument(
            'verbosity',
            default_value='3',
            description='gz sim verbosity, 0-4.',
        ),
        DeclareLaunchArgument(
            'ardupilot_gazebo',
            default_value=default_ardupilot_gazebo,
            description='Path to a built ArduPilot/ardupilot_gazebo checkout. Its models/ '
                        'supplies model://iris_with_ardupilot and its build/ supplies the '
                        'ArduPilotPlugin.',
        ),
        OpaqueFunction(function=_launch_gazebo),
    ])
