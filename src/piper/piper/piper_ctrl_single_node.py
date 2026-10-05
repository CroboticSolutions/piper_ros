#!/usr/bin/env python3
# -*-coding:utf8-*-
# This file controls a single robotic arm node and handles the movement of the robotic arm with a gripper.
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Empty
from piper.command_guard import CommandGuard
from piper.sdk_backend import ERR_FIELDS, create_backend, fresh
import time
import threading
import argparse
import math
from piper_msgs.msg import PiperStatusMsg, PosCmd
from piper_msgs.srv import Enable
from geometry_msgs.msg import Pose, PoseStamped
from scipy.spatial.transform import Rotation as R  # For Euler angle to quaternion conversion
from builtin_interfaces.msg import Time

class PiperRosNode(Node):
    """ROS2 node for the robotic arm"""

    def __init__(self) -> None:
        super().__init__('piper_ctrl_single_node')
        # ROS parameters
        self.declare_parameter('can_port', 'can0')
        self.declare_parameter('auto_enable', False)
        self.declare_parameter('gripper_exist', True)
        self.declare_parameter('gripper_val_mutiple', 1)
        self.declare_parameter('require_hardware_feedback', False)
        # 'pyagxarm' (default) or 'piper_sdk' (previous SDK, kept as fallback).
        self.declare_parameter('sdk_backend', 'pyagxarm')
        # pyAgxArm firmware profile; S-V1.8-6 resolves to v183.
        self.declare_parameter('agx_firmware', 'v183')
        self.sdk_backend = self.get_parameter('sdk_backend').value
        self.agx_firmware = self.get_parameter('agx_firmware').value
        self.guarded = self.get_parameter('require_hardware_feedback').value

        self.can_port = self.get_parameter('can_port').get_parameter_value().string_value
        self.auto_enable = self.get_parameter('auto_enable').get_parameter_value().bool_value
        self.gripper_exist = self.get_parameter('gripper_exist').get_parameter_value().bool_value
        self.gripper_val_mutiple = self.get_parameter('gripper_val_mutiple').get_parameter_value().integer_value
        self.gripper_val_mutiple = max(0, min(self.gripper_val_mutiple, 10))

        self.get_logger().info(f"can_port is {self.can_port}")
        self.get_logger().info(f"auto_enable is {self.auto_enable}")
        self.get_logger().info(f"gripper_exist is {self.gripper_exist}")
        self.get_logger().info(f"gripper_val_mutiple is {self.gripper_val_mutiple}")
        self.get_logger().info(f"sdk_backend is {self.sdk_backend}")
        # Publishers
        self.joint_pub = self.create_publisher(JointState, 'joint_states_single', 1)
        self.joint_feedback_pub = self.create_publisher(JointState, 'joint_states_feedback', 1)
        self.joint_ctrl_pub = self.create_publisher(JointState, 'joint_ctrl', 1)
        self.arm_status_pub = self.create_publisher(PiperStatusMsg, 'arm_status', 1)
        self.end_pose_pub = self.create_publisher(Pose, 'end_pose', 1)
        self.end_pose_stamped_pub = self.create_publisher(PoseStamped, 'end_pose_stamped', 1)
        # Polling enable service runs separately so stop/watchdog callbacks cannot be starved.
        self.can_command_lock = threading.RLock()
        self.service_group = MutuallyExclusiveCallbackGroup()
        self.motor_srv = self.create_service(Enable, 'enable_srv', self.handle_enable_service,
                                             callback_group=self.service_group)
        # Joint
        self.joint_states = JointState()
        self.joint_states.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'gripper']
        self.joint_states.position = [0.0] * 7
        self.joint_states.velocity = [0.0] * 7
        self.joint_states.effort = [0.0] * 7

        self.joint_states_feedback = JointState()
        self.joint_states_feedback.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'gripper']
        self.joint_states_feedback.position = [0.0] * 7
        self.joint_states_feedback.velocity = [0.0] * 7
        self.joint_states_feedback.effort = [0.0] * 7
        # Joint ctrl
        self.joint_ctrl = JointState()
        self.joint_ctrl.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'gripper']
        self.joint_ctrl.position = [0.0] * 7
        self.joint_ctrl.velocity = [0.0] * 7
        self.joint_ctrl.effort = [0.0] * 7
        # Enable flag
        self.__enable_flag = False
        # Create piper class and open CAN interface
        self.arm = create_backend(self.sdk_backend, self.can_port, self.gripper_exist,
                                  self.agx_firmware, self.get_logger())
        self.arm.connect()

        self.guard = CommandGuard(self.gripper_exist)
        if self.guarded:
            self.create_subscription(JointState, '/joint_states', self.measured_callback, 1)
            self.create_subscription(Bool, '/piper/hardware_ready', self.ready_callback, 1)
            self.create_subscription(Bool, '/piper/teaching', self.teaching_callback, 1)
            self.create_subscription(Empty, '/piper/stop', self.stop_callback, 1)
            self.create_timer(0.05, self.command_watchdog)

        # Start subscription thread
        self.create_subscription(PosCmd, 'pos_cmd', self.pos_callback, 1)
        self.create_subscription(JointState, 'joint_ctrl_single', self.joint_callback, 1)
        self.create_subscription(Bool, 'enable_flag', self.enable_callback, 1)

        # Daemon: after executor shutdown the ROS rate never fires again.
        self.publisher_thread = threading.Thread(target=self.publish_thread, daemon=True)
        self.publisher_thread.start()

    def destroy_node(self):
        # pyAgxArm reader/monitor threads are non-daemon; without this the process never exits.
        try:
            self.arm.close()
        except Exception as exc:
            self.get_logger().error(f'CAN backend close failed: {exc}')
        return super().destroy_node()

    def GetEnableFlag(self):
        return self.__enable_flag

    def publish_thread(self):
        """Publish messages from the robotic arm
        """
        rate = self.create_rate(200)  # 200 Hz
        enable_flag = False
        # Set timeout (seconds)
        timeout = 5
        # Record the time before entering the loop
        start_time = time.time()
        elapsed_time_flag = False
        while rclpy.ok():
            if(self.auto_enable):
                while not (enable_flag):
                    elapsed_time = time.time() - start_time
                    self.get_logger().info("--------------------")
                    enable_flag = all(self.arm.motors_enabled())
                    self.get_logger().info(f"Enable status:{enable_flag}")
                    self.arm.enable()
                    self.set_gripper_enabled(True)
                    if(enable_flag):
                        self.__enable_flag = True
                    self.get_logger().info("--------------------")
                    # Check if the timeout has been exceeded
                    if elapsed_time > timeout:
                        self.get_logger().info("Timeout....")
                        elapsed_time_flag = True
                        enable_flag = True
                        break
                    time.sleep(1)
                    pass
            if(elapsed_time_flag):
                self.get_logger().info("Automatic enable timeout, exiting program")
                rclpy.shutdown()

            if self.arm.is_ok():
                self.PublishArmState()
                self.PublishArmJointAndGripper()
                self.PublishArmCtrlAndGripper()
                self.PublishArmEndPose()
            else:
                self.get_logger().error(f"{self.can_port} is loss")
                self.get_logger().error(f"exit...")
                self.stop_callback(None)
                rclpy.shutdown()

            rate.sleep()

    def PublishArmState(self):
        status = self.arm.status()
        if status is None:
            return
        arm_status = PiperStatusMsg()
        arm_status.ctrl_mode = status.ctrl_mode
        arm_status.arm_status = status.arm_status
        arm_status.mode_feedback = status.mode_feed
        arm_status.teach_status = status.teach_status
        arm_status.motion_status = status.motion_status
        arm_status.trajectory_num = status.trajectory_num
        arm_status.err_code = status.err_code
        for field in ERR_FIELDS:
            setattr(arm_status, field, getattr(status, field))
        self.arm_status_pub.publish(arm_status)

    def float_to_ros_time(self, t: float) -> Time:
        ros_time = Time()
        ros_time.sec = int(t)
        ros_time.nanosec = int((t - ros_time.sec) * 1e9)
        return ros_time

    def PublishArmJointAndGripper(self):
        joints, motors = self.arm.joints(), self.arm.motors()
        if joints is None or motors is None:
            return
        gripper = self.arm.gripper_state() or (0.0, 0.0, 0.0)
        # Stamp is the newer of joint and motor CAN frames (previous behaviour).
        self.joint_states.header.stamp = self.float_to_ros_time(max(joints[1], motors[2]))
        position = joints[0] + [gripper[0]]
        velocity = list(motors[0])
        effort = list(motors[1]) + [gripper[1]]
        self.joint_states.position = position
        self.joint_states.velocity = velocity
        self.joint_states.effort = effort

        self.joint_states_feedback.position = list(position)
        self.joint_states_feedback.velocity = list(velocity)
        self.joint_states_feedback.effort = list(effort)
        self.joint_states_feedback.header.stamp = self.joint_states.header.stamp
        # 发布所有消息
        if any(abs(pos) > 3.5 for pos in self.joint_states_feedback.position):
            self.get_logger().warn("Joint state abnormal: value exceeds ±3.5 rad")
        else:
            self.joint_feedback_pub.publish(self.joint_states_feedback)
            self.joint_pub.publish(self.joint_states)

    def PublishArmCtrlAndGripper(self):
        joints = self.arm.joint_ctrl()
        if joints is None:
            return  # No 0x155-0x157 seen on the bus (no leader arm / other sender).
        gripper = self.arm.gripper_ctrl() or (0.0, 0.0)
        self.joint_ctrl.header.stamp = self.float_to_ros_time(max(joints[1], gripper[1]))
        self.joint_ctrl.position = joints[0] + [gripper[0]]
        if any(abs(pos) > 3.5 for pos in self.joint_ctrl.position):
            self.get_logger().warn("Joint state abnormal: value exceeds ±3.5 rad")
        else:
            self.joint_ctrl_pub.publish(self.joint_ctrl)

    def PublishArmEndPose(self):
        sample = self.arm.end_pose()
        if sample is None:
            return
        pose, new_time = sample
        # End effector pose
        endpos = Pose()
        endpos.position.x, endpos.position.y, endpos.position.z = pose[:3]
        quaternion = R.from_euler('xyz', pose[3:]).as_quat()
        endpos.orientation.x = quaternion[0]
        endpos.orientation.y = quaternion[1]
        endpos.orientation.z = quaternion[2]
        endpos.orientation.w = quaternion[3]
        self.end_pose_pub.publish(endpos)
        #  时间戳的endpose
        end_pos_stamp = PoseStamped()
        end_pos_stamp.pose = endpos
        end_pos_stamp.header.stamp = self.float_to_ros_time(new_time)
        self.end_pose_stamped_pub.publish(end_pos_stamp)

    def pos_callback(self, pos_data):
        """Callback function for subscribing to the end effector pose

        Args:
            pos_data (): The position data
        """
        if self.guarded:
            self.get_logger().warning('Cartesian command rejected: ros2_control owns this arm')
            return
        # Position keeps the previous 1 mm command resolution.
        pose = [round(v * 1000) / 1000 for v in (pos_data.x, pos_data.y, pos_data.z)]
        pose += [pos_data.roll, pos_data.pitch, pos_data.yaw]
        if(self.GetEnableFlag()):
            self.arm.move_p(pose, 50)
            if self.gripper_exist:
                self.arm.gripper_command(max(0.0, min(0.08, pos_data.gripper)))

    def joint_callback(self, joint_data):
        """Callback function for joint angles

        Args:
            joint_data (): The joint data
        """
        with self.can_command_lock:
            if not self.GetEnableFlag():
                return
            if self.guarded:
                stamp = joint_data.header.stamp.sec + joint_data.header.stamp.nanosec * 1e-9
                values = self.guard.accept(joint_data.name, joint_data.position, stamp,
                                           time.time(), time.monotonic())
                if values is None:
                    return
            else:
                mapping = dict(zip(joint_data.name, joint_data.position))
                names = [f'joint{i}' for i in range(1, 7)]
                if not all(n in mapping and math.isfinite(mapping[n]) for n in names):
                    return
                values = [mapping[n] for n in names]
                if self.gripper_exist:
                    if 'joint7' not in mapping or not math.isfinite(mapping['joint7']):
                        return
                    values.append(mapping['joint7'])
            self.arm.move_j(values[:6])  # MOVE J, 100 %
            if self.gripper_exist:
                # joint7 is one finger's displacement. CAN uses full jaw opening.
                self.arm.gripper_command(max(0.0, min(0.08, values[6] * 2)))

    def measured_callback(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.guard.set_feedback(msg.name, msg.position, stamp, time.time(), time.monotonic())

    def ready_callback(self, msg):
        self.guard.ready = msg.data
        self.guard.ready_at = time.monotonic()
        self.command_watchdog()

    def teaching_callback(self, msg):
        self.guard.set_teaching(msg.data, time.monotonic())

    def command_watchdog(self):
        if self.guard.expired(time.monotonic()):
            self.stop_callback(None)

    def stop_callback(self, _msg):
        with self.can_command_lock:
            if self.guard.stopped:
                return
            self.guard.stopped = True
            self.guard.active = False
            self.__enable_flag = False
            try:
                # Protocol quick-stop, not DisableArm (which can release the arm).
                # Never send the 0x02 reset automatically.
                self.arm.quick_stop()
            except Exception as exc:
                self.get_logger().error(f'CAN quick-stop could not be sent: {exc}')
            self.get_logger().error('Command/feedback watchdog stopped PiPER; explicit recovery required')


    def set_gripper_enabled(self, enabled):
        if not self.gripper_exist:
            return
        measured = self.arm.gripper_state()
        if not fresh(measured, 0.25):
            return  # Unknown position must never be replaced by a zero command.
        if enabled:
            self.arm.gripper_command(measured[0])
        else:
            self.arm.gripper_disable()

    def enable_callback(self, enable_flag: Bool):
        with self.can_command_lock:
            if enable_flag.data:
                if self.guarded and self.guard.stopped:
                    if not self.guard.healthy(time.monotonic()):
                        self.get_logger().error('Clear firmware fault and restore feedback before enabling')
                        return
                    self.guard.stopped = False
                self.guard.synchronized = False
                self.__enable_flag = True
                self.arm.enable()
                self.set_gripper_enabled(True)
            else:
                self.__enable_flag = False
                self.guard.active = False
                self.guard.synchronized = False
                self.arm.disable()
                self.set_gripper_enabled(False)

    def handle_enable_service(self, req, resp):
        # Separate callback group + two executor workers: polling cannot block quick-stop.
        self.enable_callback(Bool(data=req.enable_request))
        until = time.monotonic() + 5.0
        while rclpy.ok() and time.monotonic() < until:
            if req.enable_request and self.guard.stopped:
                break
            flags = self.arm.motors_enabled()
            if all(value == req.enable_request for value in flags):
                resp.enable_response = True
                return resp
            time.sleep(0.02)
        resp.enable_response = False
        return resp


def main(args=None):
    rclpy.init(args=args)
    piper_single_node = PiperRosNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(piper_single_node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        piper_single_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
