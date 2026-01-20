#!/usr/bin/env python3
"""
Launch file for AprilTag 6D Pose Estimation Node

Usage:
  ros2 launch apriltag_detector tag_localizer.launch.py
  ros2 launch apriltag_detector tag_localizer.launch.py config:=my_custom_params.yaml
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare launch arguments
    config_arg = DeclareLaunchArgument(
        'config',
        default_value='tag_localizer_params.yaml',
        description='Name of the YAML config file in the config/ directory'
    )

    # Path to config file
    config_file = PathJoinSubstitution([
        FindPackageShare('apriltag_detector'),
        'config',
        LaunchConfiguration('config')
    ])

    # Tag Localizer Node
    tag_localizer_node = Node(
        package='apriltag_detector',
        executable='tag_localizer_node',
        name='tag_localizer_node',
        output='screen',
        parameters=[config_file],
        # Uncomment for remapping topics:
        # remappings=[
        #     ('color/image_raw', '/camera/color/image_raw'),
        #     ('depth/aligned_depth_to_color', '/camera/depth/image_raw'),
        # ]
    )

    return LaunchDescription([
        config_arg,
        tag_localizer_node,
    ])
