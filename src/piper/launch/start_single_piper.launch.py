from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
import os

os.environ["RCUTILS_COLORIZED_OUTPUT"] = "1"   # 强制彩色日志

def generate_launch_description():
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Logging level (debug, info, warn, error, fatal).'
    )
    # Declare the launch arguments
    can_port_arg = DeclareLaunchArgument(
        'can_port',
        default_value='can0',
        description='CAN port to be used by the Piper node.'
    )
    auto_enable_arg = DeclareLaunchArgument(
        'auto_enable',
        default_value='true',
        description='Automatically enable the Piper node.'
    )

    rviz_ctrl_flag_arg = DeclareLaunchArgument(
        'rviz_ctrl_flag',
        default_value='false',
        description='Start rviz flag.'
    )

    gripper_exist_arg = DeclareLaunchArgument(
        'gripper_exist',
        default_value='true',
        description='gripper'
    )

    gripper_val_mutiple_arg = DeclareLaunchArgument(
        'gripper_val_mutiple',
        default_value='1',
        description='gripper'
    )

    teach_sync_arg = DeclareLaunchArgument(
        'teach_sync',
        default_value='true',
        description='Run the teach-sync node: while drag-teach is active, track the measured '
                    'pose into the controller setpoint so the arm stays put on teach exit.'
    )

    # Define the node
    # Pinned to dedicated cores 0-1 and given SCHED_FIFO priority so the CAN
    # command/feedback loop keeps its timing regardless of load from vision/
    # perception nodes (hamer, camera, etc.) sharing the rest of the machine.
    piper_node = Node(
        package='piper',
        executable='piper_single_ctrl',
        name='piper_ctrl_single_node',
        output='screen',
        prefix='taskset -c 0,1 chrt -f 40',
        ros_arguments=['--log-level', LaunchConfiguration('log_level')],
        parameters=[{
            'can_port': LaunchConfiguration('can_port'),
            'auto_enable': LaunchConfiguration('auto_enable'),
            'gripper_val_mutiple': LaunchConfiguration('gripper_val_mutiple'),
            'gripper_exist': LaunchConfiguration('gripper_exist'),
        }],
        remappings=[
            # Command input. The MoveIt/ros2_control mock setpoint stream is published by
            # joint_state_broadcaster (use_local_topics:true) on /joint_state_broadcaster/joint_states.
            # This is the SAME data that used to arrive on /joint_states, just renamed, so MoveIt
            # execution is unchanged. /joint_states is now reserved for REAL arm feedback below.
            ('joint_ctrl_single', '/joint_state_broadcaster/joint_states'),
        ]
    )

    # Real-arm feedback publisher: reads CAN joint feedback (0x2A5-0x2A7) and publishes the true
    # arm pose (joint1-6 + gripper fingers joint7/joint8) to /joint_states, so RViz/move_group/
    # robot_state_publisher follow the physical arm -- including while hand-guiding in drag-teach mode.
    piper_read_node = Node(
        package='piper',
        executable='piper_read_slave_joint',
        name='piper_read_slave_joint',
        output='screen',
        prefix='taskset -c 0,1 chrt -f 40',
        ros_arguments=['--log-level', LaunchConfiguration('log_level')],
        parameters=[{
            'can_port': LaunchConfiguration('can_port'),
            'gripper_exist': LaunchConfiguration('gripper_exist'),
        }],
    )

    # Teach-sync: while drag-teach is active (arm_status.ctrl_mode 0x02/0x06) it streams the
    # measured pose into the arm/gripper controllers so the held setpoint tracks the operator's
    # hand. On teach exit the command already equals the current pose, so the arm stays where it
    # was placed instead of snapping back to the stale commanded pose.
    piper_teach_sync_node = Node(
        package='piper',
        executable='piper_teach_sync',
        name='piper_teach_sync_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('teach_sync')),
        ros_arguments=['--log-level', LaunchConfiguration('log_level')],
        parameters=[{
            'gripper_exist': LaunchConfiguration('gripper_exist'),
        }],
    )

    # Return the LaunchDescription
    return LaunchDescription([
        log_level_arg,
        can_port_arg,
        auto_enable_arg,
        gripper_exist_arg,
        gripper_val_mutiple_arg,
        teach_sync_arg,
        piper_node,
        piper_read_node,
        piper_teach_sync_node
    ])
