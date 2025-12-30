#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    package_name = 'my_bot'
    
    # Get the path to nav2_params.yaml
    nav2_params_file = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'nav2_params.yaml'
    )

    # Launch reactive ultrasonic avoidance node (NEW - replaces complex planning)
    reactive_avoidance_node = Node(
        package=package_name,
        executable='reactive_ultrasonic_avoidance.py',
        name='reactive_ultrasonic_avoidance',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # Launch ultrasonic obstacle layer node (for visualization in costmap)
    ultrasonic_node = Node(
        package=package_name,
        executable='ultrasonic_obstacle_layer.py',
        name='ultrasonic_obstacle_layer',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # Launch Nav2
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')
        ]),
        launch_arguments={
            'use_sim_time': 'true',
            'params_file': nav2_params_file
        }.items()
    )

    return LaunchDescription([
        reactive_avoidance_node,
        ultrasonic_node,
        nav2_launch
    ])
