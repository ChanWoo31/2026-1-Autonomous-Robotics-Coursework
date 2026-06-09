#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    rviz_config = LaunchConfiguration("rviz_config")
    default_config = os.path.join(
        get_package_share_directory("final_test_9_pkg"),
        "rviz",
        "final_navigation.rviz",
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "rviz_config",
            default_value=default_config,
            description="RViz config file",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_config],
            output="screen",
        ),
    ])
