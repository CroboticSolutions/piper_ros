#!/usr/bin/env python3
"""Gazebo Sim + MoveIt combined launch (UR `ur_sim_moveit.launch.py` pattern)."""

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    IfElseSubstitution,
    LaunchConfiguration,
    PathJoinSubstitution,
    TextSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory


def _configure(context):
    no_gripper = LaunchConfiguration("no_gripper").perform(context).lower() in ("true", "1")
    launch_rviz = LaunchConfiguration("launch_rviz").perform(context).lower() in ("true", "1")

    pkg_gazebo = get_package_share_directory("piper_gazebo")
    moveit_pkg = "piper_no_gripper_moveit" if no_gripper else "piper_with_gripper_moveit"
    pkg_moveit = get_package_share_directory(moveit_pkg)

    urdf_file = os.path.join(
        pkg_gazebo,
        "urdf",
        "piper_gz_no_gripper.urdf.xacro" if no_gripper else "piper_gz.urdf.xacro",
    )
    controllers_yaml = os.path.join(
        pkg_gazebo,
        "config",
        "ros2_no_gripper_controllers.yaml" if no_gripper else "ros2_sim_controllers.yaml",
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            TextSubstitution(text=" "),
            TextSubstitution(text=urdf_file),
            TextSubstitution(text=" simulation_controllers:="),
            TextSubstitution(text=controllers_yaml),
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[{"use_sim_time": True}, robot_description],
    )

    world_to_base_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["--frame-id", "world", "--child-frame-id", "base_link"],
        parameters=[{"use_sim_time": True}],
        output="log",
    )

    gazebo_camera_frame_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "--frame-id",
            "oak_right_camera_frame",
            "--child-frame-id",
            "piper/link6/camera",
        ],
        parameters=[{"use_sim_time": True}],
        output="log",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--controller-manager", "/controller_manager"],
    )

    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller", "--controller-manager", "/controller_manager"],
        condition=UnlessCondition(LaunchConfiguration("no_gripper")),
    )

    # joint8 is a URDF <mimic> of joint7 enforced by gz_ros2_control — no
    # gripper8_controller / joint8_ctrl.py mirror node needed anymore.

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-string",
            robot_description_content,
            "-name",
            "piper",
            "-allow_renaming",
            "true",
        ],
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]
        ),
        launch_arguments={
            "gz_args": IfElseSubstitution(
                LaunchConfiguration("gazebo_gui"),
                if_value=["-r -v 4 ", LaunchConfiguration("world_file")],
                else_value=["-s -r -v 4 ", LaunchConfiguration("world_file")],
            )
        }.items(),
    )

    bridge_config = PathJoinSubstitution(
        [FindPackageShare("piper_gazebo"), "config", "piper_gz_bridge.yaml"]
    )
    gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[{"config_file": bridge_config}],
        output="screen",
    )

    pointcloud_reframe_node = Node(
        package="piper_gazebo",
        executable="pointcloud_reframe.py",
        output="screen",
        parameters=[
            {"input_topic": "/piper/camera/points"},
            {"output_topic": "/piper/camera/points_reframed"},
            {"frame_id": "oak_right_camera_optical_frame"},
            {"xyz_transform": "gazebo_camera_to_optical"},
        ],
        condition=UnlessCondition(LaunchConfiguration("no_gripper")),
    )

    moveit_stack_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_moveit, "launch", "piper_moveit_stack.launch.py")
        ),
        launch_arguments={
            "use_sim_time": "true",
            "launch_rviz": "true" if launch_rviz else "false",
        }.items(),
        condition=IfCondition(LaunchConfiguration("launch_move_group")),
    )

    return [
        gz_sim,
        gz_spawn_entity,
        robot_state_publisher_node,
        world_to_base_tf,
        gazebo_camera_frame_tf,
        gz_bridge,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
        gripper_controller_spawner,
        pointcloud_reframe_node,
        moveit_stack_launch,
    ]


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
            OpaqueFunction(function=_configure),
        ]
    )
