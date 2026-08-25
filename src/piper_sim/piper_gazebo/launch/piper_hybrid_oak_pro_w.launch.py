#!/usr/bin/env python3
"""Hybrid real OAK-D Pro W: DepthAI driver + sim-time stamp sync.

Gazebo owns the calibrated link6 → oak_* TF tree (wrist_camera:=oak_d_pro_w).
DepthAI driver stays on wall time under /oak_hw/oak. Stamp-sync republishes
/oak/... from /clock so RViz / MoveIt / grasping TF lookups succeed.

Do not enable DepthAI TF or robot_description — Piper robot_state_publisher
already publishes the camera frames.

Pointclouds default off (same as fanuc_hybrid_realsense.launch.py). Set
enable_pointcloud:=true when a consumer needs /oak/rgbd/points (ICP grasp,
edge snap). That flag is init-only — relaunch this file to change it.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_gazebo = get_package_share_directory("piper_gazebo")
    depthai_prefix = get_package_share_directory("depthai_ros_driver_v3")
    default_params = os.path.join(pkg_gazebo, "config", "rgbd_pro_w_hybrid.yaml")
    enable_pointcloud = LaunchConfiguration("enable_pointcloud")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params,
                description="DepthAI params for the namespaced hybrid driver node",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Stamp-sync node must follow Gazebo /clock",
            ),
            DeclareLaunchArgument(
                "enable_pointcloud",
                default_value="false",
                description=(
                    "Enable host RGB-D pointclouds (/oak_hw/oak/rgbd/points → "
                    "/oak/rgbd/points). Keep false unless a consumer subscribes."
                ),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(depthai_prefix, "launch", "driver.launch.py")
                ),
                launch_arguments={
                    "name": "oak",
                    "namespace": "oak_hw",
                    "camera_model": "OAK-D-PRO-W",
                    "params_file": LaunchConfiguration("params_file"),
                    "use_rviz": "false",
                    "publish_description": "false",
                    "publish_tf_from_calibration": "false",
                    "pointcloud.enable": enable_pointcloud,
                }.items(),
            ),
            Node(
                package="piper_gazebo",
                executable="hybrid_oak_stamp_sync.py",
                name="hybrid_oak_stamp_sync",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": LaunchConfiguration("use_sim_time"),
                        "input_namespace": "/oak_hw/oak",
                        "output_namespace": "/oak",
                        "enable_pointcloud": enable_pointcloud,
                    }
                ],
            ),
        ]
    )
