"""MoveIt move_group + RViz only (UR `ur_moveit.launch.py` pattern, no joint_state_publisher)."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launch_utils import DeclareBooleanLaunchArg


def _filter_planning_pipelines(moveit_dict):
    if "planning_pipelines" not in moveit_dict:
        return moveit_dict

    allowed_pipelines = ["ompl", "pilz_industrial_motion_planner"]
    pipelines_list = moveit_dict["planning_pipelines"]

    if isinstance(pipelines_list, list):
        moveit_dict["planning_pipelines"] = [
            p for p in pipelines_list if p in allowed_pipelines
        ]
    elif isinstance(pipelines_list, dict):
        filtered_pipelines = {
            k: v
            for k, v in pipelines_list.items()
            if k in allowed_pipelines
            or k in ["planning_pipelines", "default_planning_pipeline"]
        }
        if "default_planning_pipeline" not in filtered_pipelines:
            filtered_pipelines["default_planning_pipeline"] = "ompl"
        moveit_dict["planning_pipelines"] = filtered_pipelines

    return moveit_dict


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder(
        "piper", package_name="piper_with_gripper_moveit"
    ).to_moveit_configs()

    ld = LaunchDescription()

    ld.add_action(DeclareBooleanLaunchArg("debug", default_value=False))
    ld.add_action(DeclareBooleanLaunchArg("allow_trajectory_execution", default_value=True))
    ld.add_action(DeclareBooleanLaunchArg("publish_monitored_planning_scene", default_value=True))
    ld.add_action(DeclareLaunchArgument("capabilities", default_value=""))
    ld.add_action(DeclareLaunchArgument("disable_capabilities", default_value=""))
    ld.add_action(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation time from Gazebo",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "launch_rviz",
            default_value="true",
            description="Launch MoveIt RViz",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "rviz_config",
            default_value=str(moveit_config.package_path / "config/moveit.rviz"),
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "rviz_software_rendering",
            default_value="false",
            description=(
                "Force Mesa software rendering for RViz only. "
                "Do not enable when Gazebo GUI runs in the same launch."
            ),
        )
    )

    should_publish = LaunchConfiguration("publish_monitored_planning_scene")
    move_group_configuration = {
        "publish_robot_description_semantic": True,
        "allow_trajectory_execution": LaunchConfiguration("allow_trajectory_execution"),
        "capabilities": ParameterValue(LaunchConfiguration("capabilities"), value_type=str),
        "disable_capabilities": ParameterValue(
            LaunchConfiguration("disable_capabilities"), value_type=str
        ),
        "publish_planning_scene": should_publish,
        "publish_geometry_updates": should_publish,
        "publish_state_updates": should_publish,
        "publish_transforms_updates": should_publish,
        "monitor_dynamics": False,
    }

    moveit_dict = _filter_planning_pipelines(moveit_config.to_dict())
    move_group_params = [
        moveit_dict,
        move_group_configuration,
        {"use_sim_time": LaunchConfiguration("use_sim_time")},
    ]

    ld.add_action(
        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="screen",
            parameters=move_group_params,
        )
    )

    rviz_parameters = [
        moveit_config.robot_description,
        moveit_config.robot_description_semantic,
        moveit_config.planning_pipelines,
        moveit_config.robot_description_kinematics,
        moveit_config.joint_limits,
        {"use_sim_time": LaunchConfiguration("use_sim_time")},
    ]

    ld.add_action(
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2_moveit",
            output="log",
            arguments=["-d", LaunchConfiguration("rviz_config")],
            parameters=rviz_parameters,
            condition=IfCondition(LaunchConfiguration("launch_rviz")),
        )
    )

    return ld
