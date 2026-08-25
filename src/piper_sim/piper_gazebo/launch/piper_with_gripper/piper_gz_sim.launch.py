#!/usr/bin/env python3
"""Backward-compatible alias for piper_sim_moveit.launch.py."""

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
                "world_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("piper_gazebo"), "worlds", "piper.sdf"]
                ),
                description="Gazebo world file",
            ),
            DeclareLaunchArgument(
                "no_gripper",
                default_value="false",
                description="Use no-gripper MoveIt config and URDF",
            ),
            DeclareLaunchArgument(
                "launch_move_group",
                default_value="true",
                description="Launch MoveIt move_group and RViz",
            ),
            DeclareLaunchArgument(
                "wrist_camera",
                default_value="gazebo_oak",
                description="gazebo_oak | oak_d_pro_w | none",
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
                    "world_file": LaunchConfiguration("world_file"),
                    "no_gripper": LaunchConfiguration("no_gripper"),
                    "launch_move_group": LaunchConfiguration("launch_move_group"),
                    "wrist_camera": LaunchConfiguration("wrist_camera"),
                }.items(),
            ),
        ]
    )
