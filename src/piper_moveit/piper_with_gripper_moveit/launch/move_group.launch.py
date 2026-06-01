from moveit_configs_utils import MoveItConfigsBuilder
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils.launch_utils import (
    add_debuggable_node,
    DeclareBooleanLaunchArg,
)


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("piper", package_name="piper_with_gripper_moveit").to_moveit_configs()

    ld = LaunchDescription()

    # Launch arguments
    ld.add_action(DeclareBooleanLaunchArg("debug", default_value=False))
    ld.add_action(DeclareBooleanLaunchArg("allow_trajectory_execution", default_value=True))
    ld.add_action(DeclareBooleanLaunchArg("publish_monitored_planning_scene", default_value=True))
    ld.add_action(DeclareLaunchArgument("capabilities", default_value=""))
    ld.add_action(DeclareLaunchArgument("disable_capabilities", default_value=""))
    # Default false: demo.launch.py uses fake ros2_control + RViz without /clock; move_group must
    # agree with moveit_rviz.launch.py (also defaults false) or TF/planning scene diverges and
    # MotionPlanning interactive markers fail. Gazebo passes use_sim_time:=true explicitly.
    ld.add_action(DeclareBooleanLaunchArg("use_sim_time", default_value=False))

    should_publish = LaunchConfiguration("publish_monitored_planning_scene")

    # Move group configuration
    move_group_configuration = {
        "publish_robot_description_semantic": True,
        "allow_trajectory_execution": LaunchConfiguration("allow_trajectory_execution"),
        "capabilities": ParameterValue(LaunchConfiguration("capabilities"), value_type=str),
        "disable_capabilities": ParameterValue(LaunchConfiguration("disable_capabilities"), value_type=str),
        "publish_planning_scene": should_publish,
        "publish_geometry_updates": should_publish,
        "publish_state_updates": should_publish,
        "publish_transforms_updates": should_publish,
        "monitor_dynamics": False,
    }

    # Override planning pipelines to exclude CHOMP and STOMP (not available in ROS2 Jazzy)
    moveit_dict = moveit_config.to_dict()
    if "planning_pipelines" in moveit_dict:
        pipelines_list = moveit_dict["planning_pipelines"]
        # Filter out CHOMP and STOMP planners - keep only allowed ones
        allowed_pipelines = ["ompl", "pilz_industrial_motion_planner"]
        if isinstance(pipelines_list, list):
            # Filter the list to only include allowed planners
            filtered_pipelines = [p for p in pipelines_list if p in allowed_pipelines]
            moveit_dict["planning_pipelines"] = filtered_pipelines
        elif isinstance(pipelines_list, dict):
            # If it's a dict, filter by keys
            filtered_pipelines = {k: v for k, v in pipelines_list.items() 
                                  if k in allowed_pipelines or k in ["planning_pipelines", "default_planning_pipeline"]}
            if "default_planning_pipeline" not in filtered_pipelines:
                filtered_pipelines["default_planning_pipeline"] = "ompl"
            moveit_dict["planning_pipelines"] = filtered_pipelines
    
    move_group_params = [
        moveit_dict,
        move_group_configuration,
        {"use_sim_time": LaunchConfiguration("use_sim_time")},
    ]

    # Move group node
    add_debuggable_node(
        ld,
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=move_group_params,
    )

    return ld
