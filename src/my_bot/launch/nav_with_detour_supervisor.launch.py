"""
Launch file for Nav2 with Ultrasonic Detour Supervisor (v3.1)

Nav2 -> /cmd_vel_nav -> Supervisor -> /cmd_vel
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap, PushRosNamespace
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    my_bot_dir = get_package_share_directory('my_bot')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    default_params_file = os.path.join(my_bot_dir, 'config', 'nav2_params_with_ultrasonic.yaml')

    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true')
    params_file_arg = DeclareLaunchArgument('params_file', default_value=default_params_file)
    map_arg = DeclareLaunchArgument('map', default_value='')
    autostart_arg = DeclareLaunchArgument('autostart', default_value='true')
    slam_arg = DeclareLaunchArgument('slam', default_value='False')

    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    map_yaml = LaunchConfiguration('map')
    autostart = LaunchConfiguration('autostart')
    slam = LaunchConfiguration('slam')

    nav2_bringup = GroupAction([
        PushRosNamespace(''),
        SetRemap(src='/cmd_vel', dst='/cmd_vel_nav'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'params_file': params_file,
                'autostart': autostart,
                'slam': slam,
                'map': map_yaml,
            }.items()
        )
    ])

    ultrasonic_converter = Node(
        package='my_bot',
        executable='ultrasonic_scan_to_range.py',
        name='ultrasonic_scan_to_range',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'min_range': 0.02,
            'max_range': 4.0,
            'fov_rad': 0.24,
            'timeout_sec': 0.6,
        }]
    )

    detour_supervisor = Node(
        package='my_bot',
        executable='ultrasonic_detour_supervisor.py',
        name='ultrasonic_detour_supervisor',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,

            # geometry
            'robot_radius_m': 0.20,
            'safety_margin_m': 0.10,

            # trigger
            'avoid_extra_m': 0.25,

            # NEW: corner trigger
            'corner_extra_m': 0.05,   # D_corner = D_stop + 0.05  (~0.35)
            'corner_v_min': 0.03,

            # exit conditions
            'clear_front_m': 0.90,
            'detour_clear_m': 0.80,
            'obstacle_min_m': 0.38,

            # arc tuning
            'v_max_detour': 0.12,
            'w_min': 0.20,
            'w_max': 0.70,
            'k_corr': 0.12,
            'max_arc_yaw_deg': 75.0,
            'detour_max_time_sec': 8.0,

            # forward pass
            'commit_distance_m': 0.65,
            'v_pass': 0.12,
            'wall_target_m': 0.55,
            'k_wall_pass': 1.2,
            'w_pass_max': 0.35,

            # NEW: early exit (prevents overshoot for small obstacles)
            'pass_min_distance_m': 0.30,
            'pass_clear_margin_m': 0.10,     # early clear if dObs > wall_target + 0.10
            'pass_clear_stable_ms': 200,

            # navigate side safety
            'side_nudge_enable': True,
            'side_warn_m': 0.55,
            'side_nudge_gain': 0.30,
            'side_slow_min': 0.40,

            # hysteresis
            'persistence_ms': 250,
            'stable_ms': 400,
            'cool_down_sec': 1.2,

            # odom
            'odom_topic': '/odom',
        }]
    )

    return LaunchDescription([
        use_sim_time_arg,
        params_file_arg,
        map_arg,
        autostart_arg,
        slam_arg,
        nav2_bringup,
        ultrasonic_converter,
        detour_supervisor,
    ])
