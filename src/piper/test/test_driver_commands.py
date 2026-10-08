"""Exercise the actual driver command callback with a fake SDK backend, never a CAN port.

Wire-level frames of both backends: test_sdk_backend_frames.py."""
import math
import threading
import time
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from sensor_msgs.msg import JointState
from piper.command_guard import CommandGuard
from piper.piper_ctrl_single_node import PiperRosNode


def fake_node():
    guard = CommandGuard(True)
    now = time.monotonic()
    guard.ready = True
    guard.ready_at = now
    guard.set_teaching(False, now)
    q = [math.pi/2, math.pi/4, -math.pi/2, 0.0, -math.pi/4, math.pi, .02]
    guard.set_feedback(guard.names, q, time.time(), time.time(), now)
    node = SimpleNamespace(guard=guard, guarded=True, GetEnableFlag=lambda:True,
                           can_command_lock=threading.RLock(), gripper_exist=True, arm=Mock())
    return node, q


def message(names, positions):
    msg = JointState(name=list(names), position=list(positions))
    now = time.time()
    msg.header.stamp.sec = int(now)
    msg.header.stamp.nanosec = int(now % 1 * 1e9)
    return msg


def test_real_callback_converts_named_joints_and_finger_exactly_once():
    n, q = fake_node()
    PiperRosNode.joint_callback(n, message(reversed(n.guard.names), reversed(q)))
    n.arm.move_j.assert_called_once()
    assert n.arm.move_j.call_args.args[0] == pytest.approx(q[:6])
    n.arm.gripper_command.assert_called_once()
    # joint7 is one finger; the CAN stroke is the full 0.04 m jaw opening.
    assert n.arm.gripper_command.call_args.args[0] == pytest.approx(0.04)


def test_real_callback_cannot_default_missing_joints_to_zero_or_bypass_teach():
    n, q = fake_node()
    PiperRosNode.joint_callback(n, message(n.guard.names[:-1], q[:-1]))
    n.guard.set_teaching(True, time.monotonic())
    PiperRosNode.joint_callback(n, message(n.guard.names, q))
    n.arm.move_j.assert_not_called()


def test_gripper_enable_preserves_measured_opening():
    arm = Mock()
    arm.gripper_state.return_value = (0.043, 1.0, time.time())
    n = SimpleNamespace(gripper_exist=True, arm=arm)
    PiperRosNode.set_gripper_enabled(n, True)
    arm.gripper_command.assert_called_once_with(0.043)
    arm.reset_mock()
    arm.gripper_state.return_value = (0.043, 1.0, 0.0)  # stale measurement
    PiperRosNode.set_gripper_enabled(n, True)
    PiperRosNode.set_gripper_enabled(n, False)
    arm.gripper_command.assert_not_called()
    arm.gripper_disable.assert_not_called()


def stoppable_node():
    n, q = fake_node()
    n._PiperRosNode__enable_flag = True
    n.GetEnableFlag = lambda: n._PiperRosNode__enable_flag
    n.get_logger = Mock
    n.hold = lambda latch, reason: PiperRosNode.hold(n, latch, reason)
    PiperRosNode.joint_callback(n, message(n.guard.names, q))
    n.arm.reset_mock()
    return n, q


def test_stop_request_holds_without_any_can_frame():
    # Protocol quick-stop is a damped e-stop under which the arm sinks; DisableArm releases it.
    n, q = stoppable_node()
    PiperRosNode.stop_callback(n, None)
    assert n.arm.method_calls == []
    assert n.guard.stopped
    PiperRosNode.joint_callback(n, message(n.guard.names, q))
    n.arm.move_j.assert_not_called()


def test_command_timeout_holds_and_resumes_only_at_measured_pose():
    n, q = stoppable_node()
    n.guard.command_at -= 1.0  # ros2_control stopped; feedback is still fresh
    PiperRosNode.command_watchdog(n)
    assert n.arm.method_calls == []
    assert not n.guard.stopped and not n.guard.active
    far = list(q)
    far[0] += 0.1
    PiperRosNode.joint_callback(n, message(n.guard.names, far))
    n.arm.move_j.assert_not_called()
    PiperRosNode.joint_callback(n, message(n.guard.names, q))
    n.arm.move_j.assert_called_once()


def test_stale_feedback_latches_hold_without_e_stop():
    n, _ = stoppable_node()
    n.guard.feedback_at -= 1.0
    PiperRosNode.command_watchdog(n)
    assert n.arm.method_calls == []
    assert n.guard.stopped
