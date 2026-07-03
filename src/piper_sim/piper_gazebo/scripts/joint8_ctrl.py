#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from control_msgs.msg import JointTrajectoryControllerState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class GripperMirrorController(Node):
    def __init__(self):
        super().__init__('gripper_mirror_controller')
        self.declare_parameter('joint7_name', 'joint7')
        self.declare_parameter('mirror_joint_name', 'joint8')
        self.declare_parameter('mirror_sign', -1.0)
        self.declare_parameter('controller_state_topic', '/gripper_controller/controller_state')
        self.declare_parameter('mirror_command_topic', '/gripper8_controller/joint_trajectory')
        self.declare_parameter('trajectory_time_sec', 0.2)

        self.joint7_name = self.get_parameter('joint7_name').value
        self.mirror_joint_name = self.get_parameter('mirror_joint_name').value
        self.mirror_sign = float(self.get_parameter('mirror_sign').value)
        controller_state_topic = self.get_parameter('controller_state_topic').value
        mirror_command_topic = self.get_parameter('mirror_command_topic').value
        self.trajectory_time_sec = float(self.get_parameter('trajectory_time_sec').value)

        self.joint7_position = None
        self.create_subscription(
            JointTrajectoryControllerState,
            controller_state_topic,
            self.controller_state_cb,
            10,
        )
        self.publisher = self.create_publisher(JointTrajectory, mirror_command_topic, 10)
        self.timer = self.create_timer(0.02, self.publish_joint8_command)

    def controller_state_cb(self, msg: JointTrajectoryControllerState):
        try:
            joint_index = msg.joint_names.index(self.joint7_name)
        except ValueError:
            self.get_logger().warn(f"{self.joint7_name} not found in {msg.joint_names}")
            return
        if joint_index >= len(msg.reference.positions):
            return
        self.joint7_position = msg.reference.positions[joint_index]

    def publish_joint8_command(self):
        if self.joint7_position is None:
            return

        traj_msg = JointTrajectory()
        traj_msg.joint_names = [self.mirror_joint_name]

        point = JointTrajectoryPoint()
        point.positions = [self.mirror_sign * self.joint7_position]
        sec = int(self.trajectory_time_sec)
        point.time_from_start.sec = sec
        point.time_from_start.nanosec = int((self.trajectory_time_sec - sec) * 1e9)
        traj_msg.points.append(point)

        self.publisher.publish(traj_msg)

    def spin_once_sync(self):
        rclpy.spin_once(self, timeout_sec=0.0)


def main(args=None):
    rclpy.init(args=args)
    node = GripperMirrorController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
