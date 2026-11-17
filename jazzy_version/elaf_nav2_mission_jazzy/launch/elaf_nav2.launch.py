"""One launch: existing simulator, minimal Nav2 servers, bridge, and lap mission."""

import math
from pathlib import Path
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap


def start(context):

    share = Path(get_package_share_directory('elaf_nav2_mission_jazzy'))
    gym = Path(get_package_share_directory('f1tenth_gym_ros'))

    params = share / 'config/nav2_params.yaml'
    cfg = yaml.safe_load(params.read_text())

    # --------------------------------------------------
    # SPEED CONFIGURATION
    # No artificial maximum speed limit
    # --------------------------------------------------

    value = LaunchConfiguration('max_speed').perform(context)

    speed = (
        float(value)
        if value
        else cfg['controller_server']['ros__parameters']['FollowPath']['vx_max']
    )

    # Only reject invalid or non-positive speeds
    if not math.isfinite(speed) or speed <= 0.0:
        raise ValueError('max_speed must be a positive, finite number')

    # --------------------------------------------------
    # SIMULATOR CONFIGURATION
    # --------------------------------------------------

    sim = yaml.safe_load(
        (gym / 'config/sim.yaml').read_text()
    )['bridge']['ros__parameters']

    if (
        'levine_obs' not in sim['map_path']
        or sim['vehicle_params'] != 'f1tenth'
        or sim['scale'] != 1.0
        or sim['use_sim_time']
    ):
        raise ValueError(
            'This profile needs the existing '
            'levine_obs/f1tenth/scale=1/wall-time simulator config'
        )

    # --------------------------------------------------
    # TEMPORARY BEHAVIOR TREES
    # --------------------------------------------------

    temporary = tempfile.TemporaryDirectory(prefix='elaf_nav2_')

    paths = {}

    for kind, compute in [
        (
            'pose',
            '<ComputePathToPose goal="{goal}" path="{path}" '
            'planner_id="GridBased"/>'
        ),
        (
            'poses',
            '<ComputePathThroughPoses goals="{goals}" path="{path}" '
            'planner_id="GridBased"/>'
        ),
    ]:

        path = Path(temporary.name) / (kind + '.xml')

        path.write_text(
            '''<root BTCPP_format="4" main_tree_to_execute="MainTree">
<BehaviorTree ID="MainTree">
<RecoveryNode number_of_retries="5">
<PipelineSequence>
<GoalUpdatedController>'''
            + compute
            + '''</GoalUpdatedController>
<FollowPath
    path="{path}"
    controller_id="FollowPath"
    goal_checker_id="goal_checker"/>
</PipelineSequence>
<Sequence>
<ClearEntireCostmap
    service_name="local_costmap/clear_entirely_local_costmap"/>
<ClearEntireCostmap
    service_name="global_costmap/clear_entirely_global_costmap"/>
<BackUp
    backup_dist="0.25"
    backup_speed="0.15"
    time_allowance="5.0"/>
</Sequence>
</RecoveryNode>
</BehaviorTree>
</root>'''
        )

        paths[kind] = str(path)

    # --------------------------------------------------
    # RVIZ CONFIGURATION
    # --------------------------------------------------

    rviz = yaml.safe_load(
        (gym / 'config/rviz/gym_bridge.rviz').read_text()
    )

    displays = rviz['Visualization Manager']['Displays']

    for name in ('local_costmap', 'global_costmap'):

        displays.append({
            'Class': 'rviz_default_plugins/Map',
            'Name': name,
            'Enabled': True,
            'Value': True,
            'Alpha': 0.4,
            'Color Scheme': 'costmap',
            'Topic': {
                'Value': '/' + name + '/costmap',
                'Depth': 1,
                'Durability Policy': 'Transient Local',
                'Reliability Policy': 'Reliable',
            },
        })

    displays.append({
        'Class': 'rviz_default_plugins/Path',
        'Name': 'Nav2 plan',
        'Enabled': True,
        'Value': True,
        'Color': '30; 220; 60',
        'Topic': {
            'Value': '/plan',
            'Depth': 1,
            'Reliability Policy': 'Reliable',
        },
    })

    rviz_path = Path(temporary.name) / 'view.rviz'

    rviz_path.write_text(
        yaml.safe_dump(rviz, sort_keys=False)
    )

    def cleanup(_context):
        temporary.cleanup()
        return []

    # --------------------------------------------------
    # LAUNCH COMPONENTS
    # --------------------------------------------------

    return [

        RegisterEventHandler(
            OnShutdown(
                on_shutdown=[
                    OpaqueFunction(function=cleanup)
                ]
            )
        ),

        # Existing F1TENTH simulator
        GroupAction(
            actions=[

                SetRemap(
                    src='/cmd_vel',
                    dst='/elaf/disabled_teleop'
                ),

                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        str(gym / 'launch/gym_bridge_launch.py')
                    ),
                    launch_arguments={
                        'num_agent': '1',
                        'open_foxglove': 'false',
                        'foxglove_port': LaunchConfiguration(
                            'foxglove_port'
                        ),
                    }.items(),
                ),
            ]
        ),

        # MPPI controller
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            parameters=[
                str(params),
                {'FollowPath.vx_max': speed}
            ],
            remappings=[
                ('cmd_vel', '/elaf/cmd_vel')
            ],
            output='screen',
        ),

        # Hybrid A* planner
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            parameters=[str(params)],
            output='screen',
        ),

        # Navigation behaviors
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            parameters=[str(params)],
            remappings=[
                ('cmd_vel', '/elaf/cmd_vel')
            ],
            output='screen',
        ),

        # Behavior tree navigator
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            parameters=[
                str(params),
                {
                    'default_nav_to_pose_bt_xml': paths['pose'],
                    'default_nav_through_poses_bt_xml': paths['poses'],
                }
            ],
            output='screen',
        ),

        # Lifecycle manager
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            parameters=[
                {
                    'autostart': True,
                    'node_names': [
                        'controller_server',
                        'planner_server',
                        'behavior_server',
                        'bt_navigator',
                    ],
                }
            ],
            output='screen',
        ),

        # Ackermann steering bridge
        Node(
            package='elaf_nav2_mission_jazzy',
            executable='ackermann_bridge',
            name='ackermann_bridge',
            parameters=[
                str(params),
                {'max_speed': speed}
            ],
            output='screen',
        ),

        # Automatic lap mission
        Node(
            package='elaf_nav2_mission_jazzy',
            executable='mission',
            name='mission',
            parameters=[str(params)],
            condition=IfCondition(
                LaunchConfiguration('autostart_mission')
            ),
            output='screen',
        ),

        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='elaf_rviz',
            arguments=[
                '-d',
                str(rviz_path)
            ],
            additional_env={
                'GSETTINGS_SCHEMA_DIR':
                    '/usr/share/glib-2.0/schemas'
            },
            condition=IfCondition(
                LaunchConfiguration('rviz')
            ),
            output='screen',
        ),
    ]


def generate_launch_description():

    defaults = dict(
        max_speed='',
        autostart_mission='true',
        rviz='true',
        foxglove_port='8765'
    )

    return LaunchDescription([
        *[
            DeclareLaunchArgument(k, default_value=v)
            for k, v in defaults.items()
        ],
        OpaqueFunction(function=start)
    ])