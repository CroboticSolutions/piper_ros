#!/usr/bin/env python3
"""Cancel active actions in drag teach and seed a fresh measured hold on exit.

The CAN command guard independently suppresses commands throughout teaching and
requires pose agreement on exit. This node never publishes hardware commands.
"""
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from action_msgs.srv import CancelGoal


class PiperTeachSyncNode(Node):
    def __init__(self):
        super().__init__('piper_teach_sync_node')
        arm = self.declare_parameter('arm_controller', 'arm_controller').value
        gripper = self.declare_parameter('gripper_controller', 'gripper_controller').value
        self.gripper_exist = self.declare_parameter('gripper_exist', True).value
        rate = self.declare_parameter('publish_rate', 30.0).value
        self.point_time = self.declare_parameter('point_time', 0.1).value
        self.arm_joints = self.declare_parameter('arm_joints', [f'joint{i}' for i in range(1, 7)]).value
        self.gripper_joints = self.declare_parameter('gripper_joints', ['joint7']).value
        self.arm_pub = self.create_publisher(JointTrajectory, f'/{arm}/joint_trajectory', 1)
        self.grip_pub = self.create_publisher(JointTrajectory, f'/{gripper}/joint_trajectory', 1)
        self.cancel_clients = [self.create_client(CancelGoal, f'/{arm}/follow_joint_trajectory/_action/cancel_goal')]
        if self.gripper_exist:
            self.cancel_clients.append(self.create_client(CancelGoal, f'/{gripper}/follow_joint_trajectory/_action/cancel_goal'))
        self.teaching = False
        self.last_teach = -float('inf')
        self.sample = None
        self.cancel_pending = False
        self.create_subscription(Bool, '/piper/teaching', self.teach_cb, 1)
        self.create_subscription(JointState, '/joint_states', self.joint_cb, 1)
        self.create_timer(1.0 / max(1.0, rate), self.tick)

    def joint_cb(self, msg):
        self.sample = msg

    def teach_cb(self, msg):
        entering = msg.data and not self.teaching
        leaving = self.teaching and not msg.data
        self.teaching = msg.data
        self.last_teach = time.monotonic()
        if entering:
            self.cancel_pending = True
            self.get_logger().info('Teach entered: canceling controller actions')
        if entering or leaving:
            self.tick(force_hold=True)

    def tick(self, force_hold=False):
        if time.monotonic() - self.last_teach > 0.25:
            return
        if self.cancel_pending:
            available = all(c.service_is_ready() for c in self.cancel_clients)
            if available:
                for client in self.cancel_clients:
                    client.call_async(CancelGoal.Request())  # zero UUID/time: cancel all
                self.cancel_pending = False
        if not (self.teaching or force_hold):
            return
        msg = self.sample
        if msg is None or len(msg.name) != len(msg.position):
            return
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if not 0 <= time.time() - stamp <= 0.25:
            return
        positions = dict(zip(msg.name, msg.position))
        for names, pub in [(self.arm_joints, self.arm_pub)] + (
                [(self.gripper_joints, self.grip_pub)] if self.gripper_exist else []):
            if not all(n in positions for n in names):
                continue
            trajectory = JointTrajectory()
            trajectory.joint_names = list(names)
            pt = JointTrajectoryPoint()
            pt.positions = [positions[n] for n in names]
            pt.time_from_start = Duration(sec=int(self.point_time),
                                         nanosec=int(self.point_time % 1 * 1e9))
            trajectory.points = [pt]
            pub.publish(trajectory)


def main(args=None):
    rclpy.init(args=args)
    node = PiperTeachSyncNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
