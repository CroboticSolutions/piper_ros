from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launch_utils import DeclareBooleanLaunchArg, add_debuggable_node


def _moveit_config(wrist_camera: str):
    urdf_xacro = (
        Path(get_package_share_directory("piper_no_gripper_moveit"))
        / "config"
        / "piper_welding_gun.urdf.xacro"
    )
    return (
        MoveItConfigsBuilder("piper", package_name="piper_no_gripper_moveit")
        .robot_description(
            file_path=str(urdf_xacro),
            mappings={"simulation": "false", "wrist_camera": wrist_camera},
        )
        .robot_description_semantic(file_path="config/piper_welding_gun.srdf")
        # Same pipelines as the Gazebo welding profile.
        .planning_pipelines(pipelines=["ompl", "pilz_industrial_motion_planner"],
                            default_planning_pipeline="ompl")
        .to_moveit_configs()
    )


def _launch_setup(context, *args, **kwargs):
    # Selects the wrist-mounted camera's hand-eye macro in piper_welding_gun.urdf.xacro
    # ("realsense_d435" default vs. "femto_bolt") -- see that file's xacro:if/unless.
    # Resolved here (OpaqueFunction) because MoveItConfigsBuilder needs a plain
    # string at Python build time, not a deferred LaunchConfiguration substitution.
    wrist_camera = LaunchConfiguration("wrist_camera").perform(context)
    moveit_config = _moveit_config(wrist_camera)

    should_publish = LaunchConfiguration("publish_monitored_planning_scene")
    move_group_configuration = {
        "publish_robot_description_semantic": True,
        "publish_robot_description": True,
        "publish_robot_description_kinematics": True,
        "allow_trajectory_execution": LaunchConfiguration("allow_trajectory_execution"),
        "capabilities": ParameterValue(LaunchConfiguration("capabilities"), value_type=str),
        "disable_capabilities": ParameterValue(
            LaunchConfiguration("disable_capabilities"),
            value_type=str,
        ),
        "publish_planning_scene": should_publish,
        "publish_geometry_updates": should_publish,
        "publish_state_updates": should_publish,
        "publish_transforms_updates": should_publish,
        "monitor_dynamics": False,
        "use_sim_time": False,
    }

    # add_debuggable_node() calls ld.add_action(...) -- give it something with
    # that method and unwrap to a plain action list for OpaqueFunction's return.
    class _ActionSink:
        def __init__(self):
            self.actions = []

        def add_action(self, action):
            self.actions.append(action)

    sink = _ActionSink()
    # world -> base_link is already present in the shared URDF.
    sink.add_action(
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="both",
            parameters=[moveit_config.robot_description, {"use_sim_time": False}],
        )
    )
    add_debuggable_node(
        sink,
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict(), move_group_configuration],
    )
    sink.add_action(
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="log",
            arguments=["-d", str(moveit_config.package_path / "config/moveit.rviz")],
            parameters=[
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.planning_pipelines,
                moveit_config.robot_description_kinematics,
            ],
            condition=IfCondition(LaunchConfiguration("use_rviz")),
        )
    )
    sink.add_action(
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[
                moveit_config.robot_description,
                str(moveit_config.package_path / "config/ros2_welding_gun_controllers.yaml"),
            ],
            output="screen",
        )
    )
    sink.add_action(
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        )
    )
    sink.add_action(
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["arm_controller", "--controller-manager", "/controller_manager"],
        )
    )
    return sink.actions


def generate_launch_description():
    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument("wrist_camera", default_value="realsense_d435"))
    ld.add_action(DeclareBooleanLaunchArg("debug", default_value=False))
    ld.add_action(DeclareBooleanLaunchArg("use_rviz", default_value=True))
    ld.add_action(DeclareBooleanLaunchArg("allow_trajectory_execution", default_value=True))
    ld.add_action(DeclareBooleanLaunchArg("publish_monitored_planning_scene", default_value=True))
    ld.add_action(DeclareLaunchArgument("capabilities", default_value=""))
    ld.add_action(DeclareLaunchArgument("disable_capabilities", default_value=""))
    ld.add_action(OpaqueFunction(function=_launch_setup))
    return ld
