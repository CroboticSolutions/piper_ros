#!/usr/bin/env python3
"""Backward-compatible alias for piper_sim_moveit.launch.py."""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("piper_gazebo"),
                            "launch",
                            "piper_with_gripper",
                            "piper_sim_moveit.launch.py",
                        ]
                    )
                )
            )
        ]
    )
