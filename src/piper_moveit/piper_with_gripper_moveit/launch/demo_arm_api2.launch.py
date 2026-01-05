from moveit_configs_utils import MoveItConfigsBuilder
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
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
    ld.add_action(DeclareBooleanLaunchArg("gui", default_value=True))
    ld.add_action(DeclareLaunchArgument("capabilities", default_value=""))
    ld.add_action(DeclareLaunchArgument("disable_capabilities", default_value=""))

    should_publish = LaunchConfiguration("publish_monitored_planning_scene")

    # Move group configuration with necessary parameters for arm_api2
    move_group_configuration = {
        "publish_robot_description_semantic": True,
        "publish_robot_description": True,
        "publish_robot_description_kinematics": True,
        "allow_trajectory_execution": LaunchConfiguration("allow_trajectory_execution"),
        "capabilities": ParameterValue(LaunchConfiguration("capabilities"), value_type=str),
        "disable_capabilities": ParameterValue(LaunchConfiguration("disable_capabilities"), value_type=str),
        "publish_planning_scene": should_publish,
        "publish_geometry_updates": should_publish,
        "publish_state_updates": should_publish,
        "publish_transforms_updates": should_publish,
        "monitor_dynamics": False,
    }

    move_group_params = [
        moveit_config.to_dict(),
        move_group_configuration,
    ]

    # Move group node
    add_debuggable_node(
        ld,
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=move_group_params,
    )

    # Joint state publisher (with or without GUI)
    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        parameters=[{"publish_rate": 200}],
        condition=UnlessCondition(LaunchConfiguration("gui"))
    )

    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        parameters=[{"publish_rate": 200}],
        condition=IfCondition(LaunchConfiguration("gui"))
    )

    # Robot state publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[moveit_config.robot_description],
    )

    # RViz
    rviz_parameters = [
        moveit_config.planning_pipelines,
        moveit_config.robot_description_kinematics,
    ]

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="log",
        arguments=["-d", str(moveit_config.package_path / "config/moveit.rviz")],
        parameters=rviz_parameters,
    )

    # ROS2 control node
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            moveit_config.robot_description,
            str(moveit_config.package_path / "config/ros2_controllers.yaml"),
        ],
        output="screen",
    )

    # Joint state broadcaster
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    # Arm controller spawner
    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--controller-manager", "/controller_manager"],
    )

    ld.add_action(joint_state_publisher_node)
    ld.add_action(joint_state_publisher_gui_node)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(rviz_node)
    ld.add_action(ros2_control_node)
    ld.add_action(joint_state_broadcaster_spawner)
    ld.add_action(arm_controller_spawner)

    return ld
