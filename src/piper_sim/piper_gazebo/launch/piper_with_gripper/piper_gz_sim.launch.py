#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "gz_gui",
            default_value="true",
            description="Start Gazebo GUI",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_rviz",
            default_value="false",
            description="Launch RViz",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "world_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("piper_gazebo"), "worlds", "piper.sdf"]
            ),
            description="World file to load",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_move_group",
            default_value="true",
            description="Launch MoveIt move_group node",
        )
    )

    # Initialize Arguments
    gz_gui = LaunchConfiguration("gz_gui")
    launch_rviz = LaunchConfiguration("launch_rviz")
    world_file = LaunchConfiguration("world_file")
    launch_move_group = LaunchConfiguration("launch_move_group")

    # Get URDF via xacro using Command substitution (like UR does)
    piper_description_path = PathJoinSubstitution(
        [FindPackageShare("piper_gazebo"), "urdf", "piper_gz.urdf.xacro"]
    )

    simulation_controllers = PathJoinSubstitution(
        [FindPackageShare("piper_with_gripper_moveit"), "config", "ros2_controllers.yaml"]
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            piper_description_path,
            " ",
            "simulation_controllers:=",
            simulation_controllers,
        ]
    )

    robot_description = {"robot_description": robot_description_content}

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[{"use_sim_time": True}, robot_description],
    )

    # RViz Node (optional)
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("piper_description"), "rviz", "piper_ctrl.rviz"]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(launch_rviz),
    )

    # Gazebo Sim - conditionally add -s flag for headless mode
    def get_gz_sim_action(context):
        gz_gui_val = LaunchConfiguration("gz_gui").perform(context)
        world_file_val = LaunchConfiguration("world_file").perform(context)
        
        # Add -s flag for server-only (headless) mode if GUI is disabled
        # gz_args must be a string (not a list) for ros_gz_sim launch file
        if gz_gui_val.lower() == "false":
            gz_args_str = f"-r -s -v 4 {world_file_val}"
        else:
            # When GUI is enabled, use -r to run and specify world file
            gz_args_str = f"-r -v 4 {world_file_val}"
        
        from ament_index_python.packages import get_package_share_directory
        ros_gz_sim_share = get_package_share_directory("ros_gz_sim")
        
        return [IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [ros_gz_sim_share, "/launch/gz_sim.launch.py"]
            ),
            launch_arguments={
                "gz_args": gz_args_str,
            }.items(),
        )]
    
    gz_sim = OpaqueFunction(function=get_gz_sim_action)

    # Spawn Entity (same as UR approach)
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

    # Clock bridge
    gz_bridge_clock = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        ],
        output="screen",
    )

    # Camera bridge - bridge camera topics from Gazebo to ROS 2
    gz_bridge_camera = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/world/piper_world/model/piper/link/camera_link/sensor/camera/image@sensor_msgs/msg/Image@gz.msgs.Image",
            "/world/piper_world/model/piper/link/camera_link/sensor/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo",
        ],
        remappings=[
            ("/world/piper_world/model/piper/link/camera_link/sensor/camera/image", "/piper/camera/image_raw"),
            ("/world/piper_world/model/piper/link/camera_link/sensor/camera/camera_info", "/piper/camera/camera_info"),
        ],
        output="screen",
    )

    # Joint State Broadcaster
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    # Arm Controller
    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--controller-manager", "/controller_manager"],
    )

    # Gripper Controller
    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller", "--controller-manager", "/controller_manager"],
    )

    # Delay RViz after joint_state_broadcaster
    delay_rviz_after_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        ),
        condition=IfCondition(launch_rviz),
    )

    # MoveIt move_group launch
    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("piper_with_gripper_moveit"),
                "launch",
                "move_group.launch.py"
            ])
        ),
        launch_arguments={
            "use_sim_time": "true"
        }.items(),
        condition=IfCondition(launch_move_group),
    )

    # Delay move_group after arm controller spawns
    delay_move_group_after_arm_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=arm_controller_spawner,
            on_exit=[move_group_launch],
        ),
        condition=IfCondition(launch_move_group),
    )

    nodes_to_start = [
        robot_state_publisher_node,
        joint_state_broadcaster_spawner,
        delay_rviz_after_joint_state_broadcaster,
        arm_controller_spawner,
        gripper_controller_spawner,
        delay_move_group_after_arm_controller,
        gz_spawn_entity,
        gz_sim,
        gz_bridge_clock,
        gz_bridge_camera,
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)
