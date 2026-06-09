#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("final_test_lidar_pkg")
    params_file = os.path.join(pkg_share, "config", "lidar_escape_params.yaml")

    lidar_escape = Node(
        package="final_test_lidar_pkg",
        executable="lidar_escape_node",
        name="lidar_escape_node",
        output="screen",
        parameters=[params_file],
    )

    return LaunchDescription([lidar_escape])
