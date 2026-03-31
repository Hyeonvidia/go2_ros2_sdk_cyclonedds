# Copyright (c) 2024, RoboVerse community
# SPDX-License-Identifier: BSD-3-Clause

"""
Launch file for Go2 CycloneDDS driver.
Starts the driver node, robot state publisher, pointcloud_to_laserscan,
and optionally joystick, teleop, foxglove, SLAM, and Nav2.
"""

import os
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import (
    FrontendLaunchDescriptionSource,
    PythonLaunchDescriptionSource,
)


def generate_launch_description():
    pkg_dir = get_package_share_directory('go2_robot_sdk')

    # Paths
    urdf_file = os.path.join(pkg_dir, 'urdf', 'go2.urdf')
    rviz_config = os.path.join(pkg_dir, 'config', 'cyclonedds_config.rviz')
    joystick_config = os.path.join(pkg_dir, 'config', 'joystick.yaml')
    twist_mux_config = os.path.join(pkg_dir, 'config', 'twist_mux.yaml')
    slam_config = os.path.join(pkg_dir, 'config', 'mapper_params_online_async.yaml')
    nav2_config = os.path.join(pkg_dir, 'config', 'nav2_params.yaml')

    # Load URDF
    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    # Launch arguments
    args = [
        DeclareLaunchArgument('rviz2', default_value='true',
                              description='Launch RViz2'),
        DeclareLaunchArgument('foxglove', default_value='true',
                              description='Launch Foxglove Bridge (requires foxglove_bridge pkg)'),
        DeclareLaunchArgument('slam', default_value='true',
                              description='Launch SLAM Toolbox'),
        DeclareLaunchArgument('nav2', default_value='false',
                              description='Launch Nav2'),
        DeclareLaunchArgument('joystick', default_value='false',
                              description='Launch joystick node'),
        DeclareLaunchArgument('teleop', default_value='false',
                              description='Launch teleop twist mux'),
        DeclareLaunchArgument('z_offset', default_value='0.07',
                              description='Z-axis offset for odom/TF'),
        DeclareLaunchArgument('enable_cmd_vel', default_value='false',
                              description='Enable cmd_vel forwarding to Go2 (safety: disabled by default)'),
    ]

    # Core nodes
    nodes = [
        # Go2 CycloneDDS driver
        Node(
            package='go2_robot_sdk',
            executable='go2_driver_node',
            name='go2_driver_node',
            output='screen',
            parameters=[{
                'z_offset': LaunchConfiguration('z_offset'),
                'enable_cmd_vel': LaunchConfiguration('enable_cmd_vel'),
            }],
        ),

        # Robot state publisher (URDF → TF)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='go2_robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'robot_description': robot_description,
            }],
        ),

        # PointCloud2 → LaserScan (cloud is now in odom frame, transform via TF to base_link)
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='go2_pointcloud_to_laserscan',
            output='screen',
            remappings=[
                ('cloud_in', '/go2/cloud'),
                ('scan', 'scan'),
            ],
            parameters=[{
                'target_frame': 'base_link',
                'transform_tolerance': 0.5,
                'min_height': -0.1,
                'max_height': 0.5,
                'angle_min': -3.14159,
                'angle_max': 3.14159,
                'angle_increment': 0.0174533,
                'scan_time': 0.033,
                'range_min': 0.35,
                'range_max': 20.0,
                'use_inf': True,
                'concurrency_level': 1,
            }],
        ),

        # C++ point cloud aggregator (subscribes to Go2 native /utlidar/cloud)
        Node(
            package='lidar_processor_cpp',
            executable='pointcloud_aggregator_node',
            name='pointcloud_aggregator',
            output='screen',
            remappings=[
                ('/pointcloud/aggregated', '/go2/cloud'),
            ],
            parameters=[{
                'max_range': 20.0,
                'min_range': 0.1,
                'height_filter_min': 0.05,
                'height_filter_max': 3.0,
                'downsample_rate': 2,
                'publish_rate': 10.0
            }],
        ),
    ]

    # Optional: RViz2
    nodes.append(
        Node(
            package='rviz2',
            executable='rviz2',
            name='go2_rviz2',
            output='screen',
            condition=IfCondition(LaunchConfiguration('rviz2')),
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': False}],
        )
    )

    # TTS Node (speech_processor — speaking only, no cache)
    nodes.append(
        Node(
            package='speech_processor',
            executable='tts_node',
            name='tts_node',
            output='screen',
            parameters=[{
                'api_key': os.getenv('ELEVENLABS_API_KEY', ''),
                'provider': 'elevenlabs',
                'voice_name': 'XrExE9yKIg1WjnnlVkGX',
                'local_playback': False,
                'use_cache': False,
                'audio_quality': 'standard',
            }],
        )
    )

    # Optional: Joystick
    nodes.append(
        Node(
            package='joy',
            executable='joy_node',
            condition=IfCondition(LaunchConfiguration('joystick')),
            parameters=[joystick_config],
        )
    )
    nodes.append(
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='go2_teleop_node',
            condition=IfCondition(LaunchConfiguration('joystick')),
            parameters=[twist_mux_config],
        )
    )
    nodes.append(
        Node(
            package='twist_mux',
            executable='twist_mux',
            output='screen',
            condition=IfCondition(LaunchConfiguration('teleop')),
            parameters=[
                {'use_sim_time': False},
                twist_mux_config,
            ],
        )
    )

    # Included launches
    includes = []

    # Foxglove Bridge (optional — not available on ARM64 apt)
    try:
        foxglove_launch = os.path.join(
            get_package_share_directory('foxglove_bridge'),
            'launch', 'foxglove_bridge_launch.xml')
        includes.append(
            IncludeLaunchDescription(
                FrontendLaunchDescriptionSource(foxglove_launch),
                condition=IfCondition(LaunchConfiguration('foxglove')),
            )
        )
    except PackageNotFoundError:
        pass  # foxglove_bridge not installed, skip

    # SLAM Toolbox
    slam_launch = os.path.join(
        get_package_share_directory('slam_toolbox'),
        'launch', 'online_async_launch.py')
    includes.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_launch),
            condition=IfCondition(LaunchConfiguration('slam')),
            launch_arguments={
                'slam_params_file': slam_config,
                'use_sim_time': 'false',
            }.items(),
        )
    )

    # Nav2
    nav2_launch = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'launch', 'navigation_launch.py')
    includes.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch),
            condition=IfCondition(LaunchConfiguration('nav2')),
            launch_arguments={
                'params_file': nav2_config,
                'use_sim_time': 'false',
            }.items(),
        )
    )

    return LaunchDescription(args + nodes + includes)
