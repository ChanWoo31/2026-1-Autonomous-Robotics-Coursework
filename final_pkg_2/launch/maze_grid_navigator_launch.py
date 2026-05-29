#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="final_pkg_2",
            executable="maze_grid_navigator",
            name="maze_grid_navigator",
            output="screen",
            parameters=[{
                "planning_clearance": 0.085,
                "goal_clearance": 0.10,
                "preferred_clearance": 0.18,
                "max_linear_speed": 0.13,
                "front_stop_distance": 0.23,
                "front_critical_distance": 0.16,
                "side_guard_distance": 0.105,
            }],
        ),
    ])
