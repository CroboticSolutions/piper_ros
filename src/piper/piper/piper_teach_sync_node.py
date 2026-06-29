#!/usr/bin/env python3
# -*-coding:utf8-*-
"""Teach-sync node: keep the controller setpoint glued to the hand-placed pose.

Problem this solves
-------------------
On this rig the arm is driven by a ros2_control position controller (arm_controller, a JTC,
backed by the mock hardware). That controller permanently holds its last commanded setpoint,
and the piper driver streams that setpoint to the arm at ~200 Hz as CAN JointCtrl commands.

While drag-teach is ON (arm_status.ctrl_mode == 0x02 / 0x06) the motors are passive, so the arm
ignores those streamed commands and you can move it by hand. But the controller setpoint never
follows your hand. The instant teach is turned OFF the arm starts obeying the still-streaming
*stale* setpoint and snaps away from where you placed it.

What this node does
-------------------
While teach mode is active it continuously rewrites the controller setpoint to the arm's
*measured* pose, by publishing single-point JointTrajectory messages to the arm (and gripper)
controllers. So the held setpoint tracks your hand the whole time; when you release teach the
command already equals the current pose and the arm stays put -- no snap-back.

Because the arm ignores commands while teaching, this node never causes motion during teach; it
only removes the jump that used to happen on teach exit.

Measured pose source: /joint_states_single (the real CAN feedback from piper_ctrl_single_node):
positions [joint1..joint6, gripper]. The two gripper finger joints are derived as
joint7 = gripper/2, joint8 = -gripper/2 (same convention as piper_read_slave_joint).
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from piper_msgs.msg import PiperStatusMsg

# ctrl_mode values that mean "the operator is hand-guiding the arm":
#   0x02 teaching mode, 0x06 linked teaching input mode
TEACH_CTRL_MODES = (0x02, 0x06)


class PiperTeachSyncNode(Node):
    """Streams the measured pose into the controllers while drag-teach is active."""

    def __init__(self) -> None:
        super().__init__('piper_teach_sync_node')

        self.declare_parameter('arm_controller', 'arm_controller')
        self.declare_parameter('gripper_controller', 'gripper_controller')
        self.declare_parameter('gripper_exist', True)
        self.declare_parameter('publish_rate', 30.0)      # Hz, while teaching
        self.declare_parameter('point_time', 0.1)         # s, time_from_start of each setpoint
        self.declare_parameter('arm_joints',
                               ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'])
        self.declare_parameter('gripper_joints', ['joint7', 'joint8'])

        arm_ctrl = self.get_parameter('arm_controller').get_parameter_value().string_value
        grip_ctrl = self.get_parameter('gripper_controller').get_parameter_value().string_value
        self.gripper_exist = self.get_parameter('gripper_exist').get_parameter_value().bool_value
        self.point_time = float(self.get_parameter('point_time').value)
        rate = max(float(self.get_parameter('publish_rate').value), 1.0)
        self.arm_joints = list(self.get_parameter('arm_joints').value)
        self.gripper_joints = list(self.get_parameter('gripper_joints').value)

        self.arm_traj_pub = self.create_publisher(
            JointTrajectory, f'/{arm_ctrl}/joint_trajectory', 10)
        self.grip_traj_pub = self.create_publisher(
            JointTrajectory, f'/{grip_ctrl}/joint_trajectory', 10)

        self.in_teach = False
        self.measured = None  # list of positions: [j1..j6, gripper]

        self.create_subscription(PiperStatusMsg, '/arm_status', self.status_cb, 10)
        self.create_subscription(JointState, '/joint_states_single', self.js_cb, 10)

        self.timer = self.create_timer(1.0 / rate, self.on_timer)
        self.get_logger().info(
            f"teach-sync up: arm='{arm_ctrl}', gripper='{grip_ctrl}', "
            f"rate={rate}Hz, teach modes={[hex(m) for m in TEACH_CTRL_MODES]}")

    def status_cb(self, msg: PiperStatusMsg) -> None:
        was_teaching = self.in_teach
        self.in_teach = msg.ctrl_mode in TEACH_CTRL_MODES
        if self.in_teach and not was_teaching:
            self.get_logger().info(
                f"teach ON (ctrl_mode={hex(msg.ctrl_mode)}) -> tracking measured pose into setpoint")
        elif was_teaching and not self.in_teach:
            # Lock the setpoint to the current hand-placed pose before MoveIt control resumes.
            self.publish_setpoint()
            self.get_logger().info(
                f"teach OFF (ctrl_mode={hex(msg.ctrl_mode)}) -> locked setpoint to hand-placed pose")

    def js_cb(self, msg: JointState) -> None:
        self.measured = list(msg.position)

    def on_timer(self) -> None:
        if self.in_teach:
            self.publish_setpoint()

    def publish_setpoint(self) -> None:
        if self.measured is None or len(self.measured) < 6:
            return
        dur = Duration(sec=int(self.point_time), nanosec=int((self.point_time % 1.0) * 1e9))

        arm = JointTrajectory()
        arm.joint_names = self.arm_joints
        arm_pt = JointTrajectoryPoint()
        arm_pt.positions = [float(self.measured[i]) for i in range(6)]
        arm_pt.time_from_start = dur
        arm.points = [arm_pt]
        self.arm_traj_pub.publish(arm)

        if self.gripper_exist and len(self.measured) >= 7 and len(self.gripper_joints) == 2:
            stroke = float(self.measured[6])
            grip = JointTrajectory()
            grip.joint_names = self.gripper_joints
            grip_pt = JointTrajectoryPoint()
            grip_pt.positions = [stroke / 2.0, -stroke / 2.0]
            grip_pt.time_from_start = dur
            grip.points = [grip_pt]
            self.grip_traj_pub.publish(grip)


def main(args=None):
    rclpy.init(args=args)
    node = PiperTeachSyncNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
