#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def delayed(delay_seconds, action):
    return TimerAction(period=delay_seconds, actions=[action])


def generate_launch_description():
    pkg_share = get_package_share_directory("final_test_4_pkg")
    mapper_params = os.path.join(
        pkg_share,
        "config",
        "mapper_params_online_async.yaml",
    )
    nav2_params = os.path.join(
        pkg_share,
        "config",
        "nav2_params_corridor_v9_safety.yaml",
    )
    rviz_launch = os.path.join(pkg_share, "launch", "rviz_launch.py")

    static_laser_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_link_to_laser_static_tf",
        arguments=[
            "--x", "0",
            "--y", "0",
            "--z", "0",
            "--roll", "0",
            "--pitch", "0",
            "--yaw", "3.14159",
            "--frame-id", "base_link",
            "--child-frame-id", "laser",
        ],
        output="screen",
    )

    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("slam_toolbox"),
                "launch",
                "online_async_launch.py",
            )
        ),
        launch_arguments={
            "slam_params_file": mapper_params,
        }.items(),
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("nav2_bringup"),
                "launch",
                "navigation_launch.py",
            )
        ),
        launch_arguments={
            "params_file": nav2_params,
        }.items(),
    )

    safety_filter = Node(
        package="final_test_4_pkg",
        executable="cmd_vel_safety_filter_v9",
        name="cmd_vel_safety_filter",
        output="screen",
    )

    maze_explorer = Node(
        package="final_test_4_pkg",
        executable="maze_explorer",
        name="maze_explorer",
        output="screen",
    )

    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rviz_launch),
    )

    return LaunchDescription([
        static_laser_tf,
        delayed(1.0, slam_toolbox),
        delayed(4.0, nav2),
        delayed(8.0, safety_filter),
        delayed(9.0, maze_explorer),
        delayed(10.0, rviz),
    ])
