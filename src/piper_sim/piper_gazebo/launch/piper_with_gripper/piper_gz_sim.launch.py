#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
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
from launch_ros.parameter_descriptions import ParameterValue
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

    declared_arguments.append(
        DeclareLaunchArgument(
            "controller_spawn_delay",
            default_value="15.0",
            description="Seconds to wait after piper spawn before starting controller spawners (gz_ros2_control init time)",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "controller_manager_name",
            default_value="/piper/controller_manager",
            description="Controller manager node name (gz_ros2_control often uses /<model_name>/controller_manager)",
        )
    )

    # Initialize Arguments
    gz_gui = LaunchConfiguration("gz_gui")
    launch_rviz = LaunchConfiguration("launch_rviz")
    world_file = LaunchConfiguration("world_file")
    launch_move_group = LaunchConfiguration("launch_move_group")
    controller_spawn_delay = LaunchConfiguration("controller_spawn_delay", default="15.0")
    controller_manager_name = LaunchConfiguration("controller_manager_name", default="/piper/controller_manager")

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

    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

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

    # Spawn Coke model from local model folder
    coke_spawn = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-file",
            "/root/gazebo_models/Coke/model.sdf",
            "-name",
            "coke",
            "-x",
            "0.6",
            "-y",
            "-0.15",
            "-z",
            "10",
        ],
    )

    # Spawn Banana model from local model folder
    banana_spawn = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-file",
            "/root/gazebo_models/Banana for Scale/model.sdf",
            "-name",
            "banana",
            "-x",
            "0.5",
            "-y",
            "0.15",
            "-z",
            "10",
        ],
    )

    # Spawn ArUco Marker model
    aruco_marker_spawn = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-file",
            "/root/gazebo_models/ArUco_Marker_5x5_1000_0/model.sdf",
            "-name",
            "aruco_marker",
            "-x",
            "0.2",
            "-y",
            "0.0",
            "-z",
            "0.01",  # Increased z to ensure it's above ground
        ],
    )


    bridge_config = PathJoinSubstitution(
        [FindPackageShare("piper_gazebo"), "config", "piper_gz_bridge.yaml"]
    )

    gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output="screen",
        parameters=[{
            "config_file": bridge_config,
            # optional:
            # "expand_gz_topic_names": False,
            # "override_timestamps_with_wall_time": False,
        }],
    )

    pointcloud_reframe_node = Node(
        package="piper_gazebo",
        executable="pointcloud_reframe.py",
        output="screen",
        parameters=[
            {"input_topic": "/piper/camera/points"},
            {"output_topic": "/piper/camera/points_reframed"},
            {"frame_id": "camera_link"},
        ],
    )



    # Joint State Broadcaster
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", controller_manager_name],
        parameters=[{"use_sim_time": True}],
    )

    # Arm Controller
    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--controller-manager", controller_manager_name],
        parameters=[{"use_sim_time": True}],
    )

    # Gripper Controller
    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller", "--controller-manager", controller_manager_name],
        parameters=[{"use_sim_time": True}],
    )

    # Start controller spawners only after piper is spawned, then wait for gz_ros2_control to bring up controller_manager
    # gz_ros2_control initializes asynchronously; delay ensures controller_manager services are available
    spawners_after_piper = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=gz_spawn_entity,
            on_exit=[
                TimerAction(
                    period=controller_spawn_delay,
                    actions=[
                        joint_state_broadcaster_spawner,
                        arm_controller_spawner,
                        gripper_controller_spawner,
                    ],
                ),
            ],
        ),
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
        SetEnvironmentVariable(
            name="GZ_SIM_RESOURCE_PATH",
            value="/root/gazebo_models:${GZ_SIM_RESOURCE_PATH}",
        ),
        robot_state_publisher_node,
        spawners_after_piper,
        delay_rviz_after_joint_state_broadcaster,
        delay_move_group_after_arm_controller,
        gz_spawn_entity,
        coke_spawn,
        banana_spawn,
        aruco_marker_spawn,
        gz_sim,
        gz_bridge,
        pointcloud_reframe_node,
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)
