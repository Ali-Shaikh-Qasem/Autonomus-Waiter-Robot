"""
Launch file for Nav2 with Ultrasonic Detour Supervisor

This launch file sets up:
1. Nav2 bringup with cmd_vel remapped to /cmd_vel_nav
2. Ultrasonic scan-to-range converter
3. Detour supervisor (reads /cmd_vel_nav, publishes /cmd_vel)

Topology:
  Nav2 Controller → /cmd_vel_nav → Supervisor → /cmd_vel → Robot
                                         ↑
                                    6 x /us/* topics

Usage:
    ros2 launch my_bot nav_with_detour_supervisor.launch.py slam:=True
    
    # Then send goals via:
    ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped "{header: {frame_id: 'map'}, pose: {position: {x: 2.0, y: 3.0, z: 0.0}}}"
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap
from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # Package directories
    my_bot_dir = get_package_share_directory('my_bot')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    # Nav2 params (can use existing or simpler config without range layer)
    default_params_file = os.path.join(
        my_bot_dir,
        'config',
        'nav2_params_with_ultrasonic.yaml'
    )

    # === Launch Arguments ===
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock'
    )

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Nav2 params file'
    )

    map_arg = DeclareLaunchArgument(
        'map',
        default_value='',
        description='Map file (not needed for SLAM)'
    )

    autostart_arg = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Auto-start nav2'
    )
    
    slam_arg = DeclareLaunchArgument(
        'slam',
        default_value='False',
        description='Run SLAM instead of localization'
    )
    
    # Supervisor parameters
    stop_distance_arg = DeclareLaunchArgument(
        'stop_distance',
        default_value='1.0',
        description='Obstacle trigger distance (m) - increased for earlier detection'
    )
    
    slow_distance_arg = DeclareLaunchArgument(
        'slow_distance',
        default_value='1.5',
        description='Start progressive slowdown distance (m)'
    )
    
    clear_distance_arg = DeclareLaunchArgument(
        'clear_distance',
        default_value='1.20',
        description='Clear path threshold (m)'
    )

    # Get configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    map_yaml = LaunchConfiguration('map')
    autostart = LaunchConfiguration('autostart')
    slam = LaunchConfiguration('slam')
    stop_distance = LaunchConfiguration('stop_distance')
    slow_distance = LaunchConfiguration('slow_distance')
    clear_distance = LaunchConfiguration('clear_distance')

    # === Nav2 Bringup with cmd_vel remap ===
    # We'll use a group action to apply remappings to all nav2 nodes
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

    # === Ultrasonic Converter ===
    ultrasonic_converter = Node(
        package='my_bot',
        executable='ultrasonic_scan_to_range.py',
        name='ultrasonic_scan_to_range',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # === Detour Supervisor ===
    detour_supervisor = Node(
        package='my_bot',
        executable='ultrasonic_detour_supervisor.py',
        name='ultrasonic_detour_supervisor',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'stop_distance': stop_distance,
            'slow_distance': slow_distance,
            'clear_distance': clear_distance,
            'persistence_ms': 150,
            'stable_ms': 300,
            'turn_speed': 0.5,
            'scan_step_deg': 15.0,
            'max_scan_deg': 90.0,
            'd_step': 0.5,
            'intermediate_timeout': 5.0,
            'cool_down_sec': 2.0,
            'filter_window': 5,
            'very_close_dist': 0.40,
            'backup_dist': 0.20,
            'min_speed_factor': 0.15,
        }]
    )

    return LaunchDescription([
        # Arguments
        use_sim_time_arg,
        params_file_arg,
        map_arg,
        autostart_arg,
        slam_arg,
        stop_distance_arg,
        slow_distance_arg,
        clear_distance_arg,
        
        # Nodes
        nav2_bringup,
        ultrasonic_converter,
        detour_supervisor,
    ])
