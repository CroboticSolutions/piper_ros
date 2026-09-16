#!/usr/bin/env python3
"""Gazebo Sim + MoveIt combined launch (UR `ur_sim_moveit.launch.py` pattern).

wrist_camera:
  gazebo_oak (default) — Gazebo RGB-D sensor + image bridge
  oak_d_pro_w — URDF OAK-D Pro W TF only; launch real DepthAI separately (hybrid)
  none — no wrist camera in URDF
"""

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
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
from moveit_configs_utils import MoveItConfigsBuilder


def _configure(context):
    no_gripper = LaunchConfiguration("no_gripper").perform(context).lower() in ("true", "1")
    welding_gun = LaunchConfiguration("welding_gun").perform(context).lower() in ("true", "1")
    no_gripper = no_gripper or welding_gun
    launch_rviz = LaunchConfiguration("launch_rviz").perform(context).lower() in ("true", "1")
    wrist_camera = LaunchConfiguration("wrist_camera").perform(context).strip()
    if welding_gun and wrist_camera != "gazebo_realsense":
        raise RuntimeError("PIPER WELDING GUN requires wrist_camera:=gazebo_realsense")
    if not welding_gun and wrist_camera == "gazebo_realsense":
        raise RuntimeError("gazebo_realsense requires welding_gun:=true")
    if wrist_camera not in ("gazebo_oak", "gazebo_realsense", "oak_d_pro_w", "none"):
        raise RuntimeError(
            f"unsupported wrist_camera={wrist_camera!r}; "
            "expected gazebo_oak, oak_d_pro_w, or none"
        )
    no_gz_camera = wrist_camera in ("oak_d_pro_w", "none")

    pkg_gazebo = get_package_share_directory("piper_gazebo")
    moveit_pkg = "piper_no_gripper_moveit" if no_gripper else "piper_with_gripper_moveit"
    pkg_moveit = get_package_share_directory(moveit_pkg)

    urdf_file = os.path.join(
        pkg_gazebo,
        "urdf",
        "piper_gz_welding_gun.urdf.xacro" if welding_gun else
        "piper_gz_no_gripper.urdf.xacro" if no_gripper else "piper_gz.urdf.xacro",
    )
    controllers_yaml = os.path.join(
        pkg_gazebo,
        "config",
        "ros2_no_gripper_controllers.yaml" if no_gripper else "ros2_sim_controllers.yaml",
    )
    bridge_yaml = os.path.join(
        pkg_gazebo,
        "config",
        "piper_gz_bridge_clock_only.yaml" if no_gz_camera else
        "piper_d435_bridge.yaml" if welding_gun else "piper_gz_bridge.yaml",
    )

    xacro_cmd = [
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        TextSubstitution(text=" "),
        TextSubstitution(text=urdf_file),
        TextSubstitution(text=" simulation_controllers:="),
        TextSubstitution(text=controllers_yaml),
    ]
    if not no_gripper or welding_gun:
        xacro_cmd.extend(
            [
                TextSubstitution(text=" wrist_camera:="),
                TextSubstitution(text=wrist_camera),
            ]
        )
    robot_description_content = Command(xacro_cmd)
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

    gazebo_camera_frame_tf = (
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=[
                "--frame-id",
                "camera_color_frame" if welding_gun else "oak_right_camera_frame",
                "--child-frame-id",
                "piper/link6/camera",
            ],
            parameters=[{"use_sim_time": True}],
            output="log",
        )
        if not no_gz_camera
        else None
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
        condition=IfCondition("false" if no_gripper else "true"),
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

    gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[{"config_file": bridge_yaml}],
        output="screen",
    )

    pointcloud_reframe_node = (
        Node(
            package="piper_gazebo",
            executable="pointcloud_reframe.py",
            output="screen",
            parameters=[
                {"input_topic": "/piper/camera/points"},
                {"output_topic": "/piper/camera/points_reframed"},
                {"frame_id": "oak_right_camera_optical_frame"},
                {"xyz_transform": "gazebo_camera_to_optical"},
            ],
            condition=IfCondition("true" if welding_gun or not no_gripper else "false"),
        )
        if not no_gz_camera
        else None
    )

    if welding_gun:
        pointcloud_reframe_node = Node(
            package="piper_gazebo", executable="d435_depth_processing.py",
            parameters=[{"use_sim_time": True}], output="screen",
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

    resource_paths = os.pathsep.join(filter(None, [
        os.environ.get("GZ_SIM_RESOURCE_PATH", ""),
        os.path.dirname(get_package_share_directory("piper_description")),
        os.path.dirname(get_package_share_directory("realsense2_description")) if welding_gun else "",
    ]))
    nodes = [
        SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", resource_paths),
        gz_sim,
        gz_spawn_entity,
        robot_state_publisher_node,
        world_to_base_tf,
    ]
    if gazebo_camera_frame_tf is not None:
        nodes.append(gazebo_camera_frame_tf)
    nodes.extend(
        [
            gz_bridge,
            joint_state_broadcaster_spawner,
            arm_controller_spawner,
            gripper_controller_spawner,
        ]
    )
    if pointcloud_reframe_node is not None:
        nodes.append(pointcloud_reframe_node)
    if welding_gun:
        # Gazebo, RSP, MoveIt and RViz use the same full robot/tool geometry.
        config = (MoveItConfigsBuilder("piper", package_name="piper_no_gripper_moveit")
            .robot_description(file_path=urdf_file, mappings={
                "simulation_controllers": controllers_yaml, "wrist_camera": wrist_camera})
            .robot_description_semantic(file_path=os.path.join(pkg_gazebo, "config", "piper_welding_gun.srdf"))
            .planning_pipelines(pipelines=["ompl"])
            .to_moveit_configs())
        nodes.append(Node(package="moveit_ros_move_group", executable="move_group",
            output="screen", parameters=[config.to_dict(), {
                "use_sim_time": True, "publish_robot_description_semantic": True,
                "publish_planning_scene": True, "publish_geometry_updates": True,
                "publish_state_updates": True, "publish_transforms_updates": True,
                "start_state_max_bounds_error": 0.05,
            }], condition=IfCondition(LaunchConfiguration("launch_move_group"))))
        if launch_rviz:
            nodes.append(Node(package="rviz2", executable="rviz2", name="rviz2_moveit",
                arguments=["-d", os.path.join(pkg_moveit, "config", "moveit.rviz")],
                parameters=[config.to_dict(), {"use_sim_time": True}],
                condition=IfCondition(LaunchConfiguration("launch_move_group"))))
    else:
        nodes.append(moveit_stack_launch)
    return nodes


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
            DeclareLaunchArgument("welding_gun", default_value="false", description="Mount PIPER WELDING GUN instead of the gripper"),
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
                description="gazebo_oak | gazebo_realsense (welding gun) | oak_d_pro_w | none",
            ),
            OpaqueFunction(function=_configure),
        ]
    )
