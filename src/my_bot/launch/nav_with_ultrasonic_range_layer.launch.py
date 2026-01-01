"""
Launch file for Nav2 navigation with ultrasonic range layer integration.

This launch file:
1. Includes Nav2 bringup with localization (AMCL) and navigation
2. Starts the ultrasonic_scan_to_range converter node
3. Ensures use_sim_time is properly set for simulation

Usage:
    ros2 launch my_bot nav_with_ultrasonic_range_layer.launch.py map:=/path/to/map.yaml
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Package directories
    my_bot_dir = get_package_share_directory('my_bot')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    # Path to our custom Nav2 params with ultrasonic range layer
    default_params_file = os.path.join(
        my_bot_dir,
        'config',
        'nav2_params_with_ultrasonic.yaml'
    )

    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Full path to the Nav2 parameters YAML file'
    )

    map_arg = DeclareLaunchArgument(
        'map',
        default_value='',
        description='Full path to map yaml file to load (REQUIRED)'
    )

    autostart_arg = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the nav2 stack'
    )
    
    slam_arg = DeclareLaunchArgument(
        'slam',
        default_value='False',
        description='Whether to run SLAM instead of localization'
    )

    # Get launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    map_yaml_file = LaunchConfiguration('map')
    autostart = LaunchConfiguration('autostart')
    slam = LaunchConfiguration('slam')

    # Include Nav2 full bringup (includes map_server, AMCL, and navigation)
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': autostart,
            'map': map_yaml_file,
            'slam': slam,
        }.items()
    )

    # Ultrasonic LaserScan to Range converter node
    ultrasonic_converter_node = Node(
        package='my_bot',
        executable='ultrasonic_scan_to_range.py',
        name='ultrasonic_scan_to_range',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'min_range': 0.02,
            'max_range': 4.0,
            'fov_rad': 0.24,
            'timeout_sec': 0.5,
            'log_rate_sec': 1.0,
        }],
    )

    return LaunchDescription([
        # Declare arguments
        use_sim_time_arg,
        params_file_arg,
        map_arg,
        autostart_arg,
        slam_arg,
        
        # Launch Nav2 navigation stack with localization
        nav2_bringup_launch,
        
        # Launch ultrasonic converter node
        ultrasonic_converter_node,
    ])
