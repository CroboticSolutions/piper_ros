from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_moveit_rviz_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("piper", package_name="piper_with_gripper_moveit").to_moveit_configs()
    rviz_launch = generate_moveit_rviz_launch(moveit_config)

    # Workaround for RViz crashes on some GPU/driver setups
    return LaunchDescription([
        SetEnvironmentVariable(name="QT_QPA_PLATFORM", value="xcb"),
        SetEnvironmentVariable(name="LIBGL_ALWAYS_SOFTWARE", value="1"),
        SetEnvironmentVariable(name="MESA_LOADER_DRIVER_OVERRIDE", value="llvmpipe"),
        rviz_launch,
    ])
