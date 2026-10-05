#!/usr/bin/env python3
"""Publish complete SocketCAN measurements; never initialize or command an arm."""
import threading
import time
import can
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from piper.can_feedback import CanFeedback


class PiperRosNode(Node):
    def __init__(self):
        super().__init__('piper_read_slave_joint')
        port = self.declare_parameter('can_port', 'can0').value
        gripper = self.declare_parameter('gripper_exist', True).value
        timeout = self.declare_parameter('feedback_timeout', 0.25).value
        self.feedback = CanFeedback(gripper, timeout)
        self.lock = threading.Lock()
        self.stopping = threading.Event()
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.ready_pub = self.create_publisher(Bool, '/piper/hardware_ready', 10)
        self.teach_pub = self.create_publisher(Bool, '/piper/teaching', 10)
        self.bus = can.Bus(interface='socketcan', channel=port, receive_own_messages=False,
                           can_filters=[{'can_id': i, 'can_mask': 0x7FF, 'extended': False}
                                        for i in self.feedback.required])
        self.thread = threading.Thread(target=self.receive, daemon=True)
        self.thread.start()
        self.create_timer(0.005, self.publish)

    def receive(self):
        try:
            while not self.stopping.is_set():
                msg = self.bus.recv(timeout=0.1)
                if msg is not None and not (msg.is_error_frame or msg.is_remote_frame):
                    with self.lock:
                        self.feedback.update(msg.arbitration_id, msg.data, msg.timestamp)
        except can.CanError as exc:
            self.get_logger().error(f'CAN reception stopped: {exc}')

    def publish(self):
        with self.lock:
            sample = self.feedback.snapshot(time.time())
        self.ready_pub.publish(Bool(data=sample is not None and sample['ready']))
        if sample is None:
            return
        self.teach_pub.publish(Bool(data=sample['teaching']))
        msg = JointState()
        msg.name = sample['names']
        msg.position = sample['positions']
        msg.velocity = sample['velocities']
        msg.effort = sample['efforts']
        stamp = sample['stamp']
        msg.header.stamp.sec = int(stamp)
        msg.header.stamp.nanosec = int((stamp - int(stamp)) * 1e9)
        # Stamp is the oldest required CAN measurement, never ROS timer time.
        self.joint_pub.publish(msg)

    def destroy_node(self):
        self.stopping.set()
        self.thread.join(timeout=1.0)
        self.bus.shutdown()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PiperRosNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
