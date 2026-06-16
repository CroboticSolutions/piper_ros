#!/usr/bin/env python3
"""Primary entry point: Gazebo Sim + MoveIt for Piper with gripper (UR-style)."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "gazebo_gui",
                default_value="true",
                description="Start Gazebo GUI",
            ),
            DeclareLaunchArgument(
                "launch_rviz",
                default_value="true",
                description="Launch MoveIt RViz",
            ),
            DeclareLaunchArgument(
                "no_gripper",
                default_value="false",
                description="Use no-gripper configuration",
            ),
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
                ),
                launch_arguments={
                    "gazebo_gui": LaunchConfiguration("gazebo_gui"),
                    "launch_rviz": LaunchConfiguration("launch_rviz"),
                    "no_gripper": LaunchConfiguration("no_gripper"),
                }.items(),
            ),
        ]
    )
