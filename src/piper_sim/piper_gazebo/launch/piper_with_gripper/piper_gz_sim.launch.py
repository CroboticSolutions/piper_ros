#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
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

    pkg_gazebo = get_package_share_directory("piper_gazebo")
    moveit_pkg = "piper_no_gripper_moveit" if no_gripper else "piper_with_gripper_moveit"
    pkg_moveit = get_package_share_directory(moveit_pkg)

    urdf_file = os.path.join(
        pkg_gazebo,
        "urdf",
        "piper_gz_no_gripper.urdf.xacro" if no_gripper else "piper_gz.urdf.xacro",
    )
    controllers_yaml = os.path.join(pkg_moveit, "config", "ros2_controllers.yaml")

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            TextSubstitution(text=" "),
            TextSubstitution(text=urdf_file),
            TextSubstitution(text=" simulation_controllers:="),
            TextSubstitution(text=controllers_yaml),
        ]
    )

    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[{"use_sim_time": True}, robot_description],
    )

    # Match generate_moveit_rviz_launch(...) used by demo.launch.py: kinematics /
    # planning pipelines must be loaded on rviz2 or MotionPlanning has no IK for
    # interactive markers. Also enable use_sim_time for Gazebo.
    moveit_config = MoveItConfigsBuilder("piper", package_name=moveit_pkg).to_moveit_configs()
    rviz_config_file = str(moveit_config.package_path / "config" / "moveit.rviz")
    rviz_moveit_parameters = [
        moveit_config.planning_pipelines,
        moveit_config.robot_description_kinematics,
        moveit_config.joint_limits,
        {"use_sim_time": True},
    ]

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=rviz_moveit_parameters,
        condition=IfCondition(LaunchConfiguration("launch_rviz")),
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]
        ),
        launch_arguments={
            "gz_args": ["-r -v 4 ", LaunchConfiguration("world_file")]
        }.items(),
    )

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

    world_to_base_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "--frame-id",
            "world",
            "--child-frame-id",
            "base_link",
        ],
        parameters=[{"use_sim_time": True}],
        output="log",
    )

    bridge_config = PathJoinSubstitution(
        [FindPackageShare("piper_gazebo"), "config", "piper_gz_bridge.yaml"]
    )
    gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output="screen",
        parameters=[{"config_file": bridge_config}],
    )

    pointcloud_reframe_node = Node(
        package="piper_gazebo",
        executable="pointcloud_reframe.py",
        output="screen",
        parameters=[
            {"input_topic": "/piper/camera/points"},
            {"output_topic": "/piper/camera/points_reframed"},
            {"frame_id": "oak_right_camera_optical_frame"},
        ],
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

    delay_rviz_after_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        ),
        condition=IfCondition(LaunchConfiguration("launch_rviz")),
    )

    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_moveit, "launch", "move_group.launch.py")
        ),
        launch_arguments={
            "use_sim_time": "true"
        }.items(),
        condition=IfCondition(LaunchConfiguration("launch_move_group")),
    )

    delay_move_group_after_arm_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=arm_controller_spawner,
            on_exit=[move_group_launch],
        ),
        condition=IfCondition(LaunchConfiguration("launch_move_group")),
    )

    return [
        robot_state_publisher_node,
        world_to_base_tf,
        joint_state_broadcaster_spawner,
        delay_rviz_after_joint_state_broadcaster,
        arm_controller_spawner,
        gripper_controller_spawner,
        delay_move_group_after_arm_controller,
        gz_spawn_entity,
        gz_sim,
        gz_bridge,
        pointcloud_reframe_node,
    ]


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "gz_gui",
            default_value="true",
            description="Start Gazebo GUI",
        ),
        DeclareLaunchArgument(
            "launch_rviz",
            default_value="false",
            description="Launch RViz",
        ),
        DeclareLaunchArgument(
            "world_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("piper_gazebo"), "worlds", "piper.sdf"]
            ),
            description="World file to load",
        ),
        DeclareLaunchArgument(
            "launch_move_group",
            default_value="true",
            description="Launch MoveIt move_group node",
        ),
        DeclareLaunchArgument(
            "no_gripper",
            default_value="true",
            description=(
                "If true: same MoveIt SRDF/controllers/RViz as "
                "`ros2 launch piper_no_gripper_moveit demo.launch.py`. "
                "Set false for with-gripper sim."
            ),
        ),
    ]

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=_configure)])
