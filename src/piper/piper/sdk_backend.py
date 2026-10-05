"""SDK-neutral PiPER access for piper_ctrl_single_node.

`pyagxarm` (default) uses AgileX pyAgxArm; `piper_sdk` keeps the previous SDK
for comparison and fallback. Both emit identical command frames for enable,
disable, MOVE J at 100 %, quick-stop and gripper enable (checked on vcan by
test/test_sdk_backend_frames.py). All values are SI: rad, m, s.
Every getter returns `None` until the first frame is received.
"""
import math
import time
from types import SimpleNamespace

STATUS_FIELDS = ('ctrl_mode', 'arm_status', 'mode_feed', 'teach_status', 'motion_status',
                 'trajectory_num', 'err_code')
ERR_FIELDS = tuple(f'joint_{i}_angle_limit' for i in range(1, 7)) + tuple(
    f'communication_status_joint_{i}' for i in range(1, 7))
# Current-derived effort convention of the previous SDK (A * k / 1000 per raw unit),
# kept so joint_states_single effort does not silently change units.
EFFORT_K = (1.18125, 1.18125, 1.18125, 0.95844, 0.95844, 0.95844)
GRIPPER_FORCE = 1.0  # N, matches the previous GripperCtrl(…, 1000, …)
MDEG = 180000.0 / math.pi


def create_backend(name, can_port, gripper, firmware='v183', logger=None):
    if name == 'pyagxarm':
        return AgxArmBackend(can_port, gripper, firmware, logger)
    if name == 'piper_sdk':
        return PiperSdkBackend(can_port, gripper)
    raise ValueError(f"sdk_backend must be 'pyagxarm' or 'piper_sdk', got {name!r}")


class AgxArmBackend:
    name = 'pyagxarm'

    def __init__(self, can_port, gripper, firmware='v183', logger=None):
        from pyAgxArm import AgxArmFactory, ArmModel, create_agx_arm_config
        self.gripper_exist = gripper
        self.firmware = firmware
        self.logger = logger
        # local_loopback: other local sockets (read node, candump, recorders) must
        # see our commands, as they did with piper_sdk.
        self.arm = AgxArmFactory.create_arm(create_agx_arm_config(
            robot=ArmModel.PIPER, firmeware_version=firmware, interface='socketcan',
            channel=can_port, local_loopback=True))
        self.effector = None

    def connect(self):
        self.arm.connect()
        if self.gripper_exist:
            self.effector = self.arm.init_effector('agx_gripper')
        self.check_firmware()

    def close(self):
        self.arm.disconnect()

    def check_firmware(self):
        from pyAgxArm import ArmModel, resolve_firmware_profile
        info = self.arm.get_firmware(timeout=1.0)
        if info is None:
            self._log('warn', 'Firmware did not answer; profile %s not verified' % self.firmware)
            return
        # Use only software_version: the raw string continues with the production
        # date (S-V1.8-6 + 260127) and would otherwise resolve to the wrong profile.
        software = info['software_version'].strip('\0 ')
        profile = resolve_firmware_profile(ArmModel.PIPER, software)
        level = 'info' if profile == self.firmware else 'error'
        self._log(level, f'Firmware {software} -> pyAgxArm profile {profile}, '
                         f'configured {self.firmware}')

    def _log(self, level, text):
        if self.logger is not None:
            getattr(self.logger, level)(text)

    def is_ok(self):
        return self.arm.is_ok() and not self.arm.has_comm_error()

    def enable(self):
        self.arm.enable()

    def disable(self):
        self.arm.disable()

    def _set_speed(self, percent):
        # pyAgxArm caches speed in its mode frame (default 50 %) and resends that frame
        # with every move_*; set_speed_percent() would emit an extra 0x151 with an
        # unset move mode. pyAgxArm 1.0.0 has no non-sending setter.
        self.arm._msg_mode.move_spd_rate_ctrl = percent

    def move_j(self, joints):
        self._set_speed(100)
        self.arm.move_j(list(joints))

    def move_p(self, pose, speed_percent):
        self._set_speed(speed_percent)
        self.arm.move_p(list(pose))

    def quick_stop(self):
        self.arm.electronic_emergency_stop()

    def gripper_command(self, stroke):
        self.effector.move_gripper_m(stroke, GRIPPER_FORCE)

    def gripper_disable(self):
        self.effector.disable_gripper()

    def motors_enabled(self):
        flags = []
        for i in range(1, 7):
            state = self.arm.get_driver_states(i)
            flags.append(bool(state and state.msg.foc_status.driver_enable_status))
        if self.gripper_exist:
            state = self.effector.get_gripper_status()
            flags.append(bool(state and state.msg.foc_status.driver_enable_status))
        return flags

    def status(self):
        s = self.arm.get_arm_status()
        if s is None:
            return None
        m = s.msg
        values = dict(ctrl_mode=int(m.ctrl_mode), arm_status=int(m.arm_status),
                      mode_feed=int(m.mode_feedback), teach_status=int(m.teach_status),
                      motion_status=int(m.motion_status), trajectory_num=int(m.trajectory_num),
                      err_code=int(m.err_code))
        values.update({f: bool(getattr(m.err_status, f)) for f in ERR_FIELDS})
        return SimpleNamespace(**values)

    def joints(self):
        j = self.arm.get_joint_angles()
        return None if j is None else (list(j.msg), j.timestamp)

    def motors(self):
        """Velocity rad/s and previous-SDK effort; stamp of the oldest motor frame."""
        states = [self.arm.get_motor_states(i) for i in range(1, 7)]
        if any(s is None for s in states):
            return None
        return ([s.msg.velocity for s in states],
                [s.msg.current * k for s, k in zip(states, EFFORT_K)],
                min(s.timestamp for s in states))

    def gripper_state(self):
        g = self.effector.get_gripper_status() if self.effector else None
        return None if g is None else (g.msg.value, g.msg.force, g.timestamp)

    def joint_ctrl(self):
        j = self.arm.get_leader_joint_angles()
        return None if j is None else (list(j.msg), j.timestamp)

    def gripper_ctrl(self):
        g = self.effector.get_gripper_ctrl_states() if self.effector else None
        return None if g is None else (g.msg.value, g.timestamp)

    def end_pose(self):
        p = self.arm.get_flange_pose()
        return None if p is None else (list(p.msg), p.timestamp)


class PiperSdkBackend:
    """Previous piper_sdk path, unchanged command frames."""
    name = 'piper_sdk'

    def __init__(self, can_port, gripper):
        from piper_sdk import C_PiperInterface
        self.gripper_exist = gripper
        # judge_flag rejects virtual buses; keep the real-interface check otherwise.
        self.piper = C_PiperInterface(can_name=can_port,
                                      judge_flag=not can_port.startswith('vcan'))

    def connect(self):
        self.piper.ConnectPort()

    def close(self):
        self.piper.DisconnectPort()

    def is_ok(self):
        return self.piper.isOk()

    def enable(self):
        self.piper.EnableArm(7)

    def disable(self):
        self.piper.DisableArm(7)

    def move_j(self, joints):
        self.piper.MotionCtrl_2(0x01, 0x01, 100)
        self.piper.JointCtrl(*(round(v * MDEG) for v in joints))

    def move_p(self, pose, speed_percent):
        x, y, z = (round(v * 1e6) for v in pose[:3])
        rx, ry, rz = (round(v * MDEG) for v in pose[3:])
        self.piper.MotionCtrl_2(0x01, 0x00, speed_percent)
        self.piper.EndPoseCtrl(x, y, z, rx, ry, rz)

    def quick_stop(self):
        self.piper.MotionCtrl_1(0x01, 0x00, 0x00)

    def gripper_command(self, stroke):
        self.piper.GripperCtrl(round(stroke * 1e6), 1000, 0x01, 0)

    def gripper_disable(self):
        g = self.gripper_state()
        if g is not None:
            self.piper.GripperCtrl(round(g[0] * 1e6), 1000, 0x02, 0)

    def motors_enabled(self):
        low = self.piper.GetArmLowSpdInfoMsgs()
        flags = [bool(getattr(low, f'motor_{i}').foc_status.driver_enable_status)
                 for i in range(1, 7)]
        if self.gripper_exist:
            flags.append(bool(
                self.piper.GetArmGripperMsgs().gripper_state.foc_status.driver_enable_status))
        return flags

    def status(self):
        s = self.piper.GetArmStatus().arm_status
        values = {f: int(getattr(s, f)) for f in STATUS_FIELDS}
        values.update({f: bool(getattr(s.err_status, f)) for f in ERR_FIELDS})
        return SimpleNamespace(**values)

    def joints(self):
        m = self.piper.GetArmJointMsgs()
        j = m.joint_state
        return [getattr(j, f'joint_{i}') / MDEG for i in range(1, 7)], m.time_stamp

    def motors(self):
        m = self.piper.GetArmHighSpdInfoMsgs()
        motors = [getattr(m, f'motor_{i}') for i in range(1, 7)]
        return [x.motor_speed / 1000 for x in motors], [x.effort / 1000 for x in motors], \
            m.time_stamp

    def gripper_state(self):
        m = self.piper.GetArmGripperMsgs()
        return m.gripper_state.grippers_angle / 1e6, m.gripper_state.grippers_effort / 1000, \
            m.time_stamp

    def joint_ctrl(self):
        m = self.piper.GetArmJointCtrl()
        j = m.joint_ctrl
        return [getattr(j, f'joint_{i}') / MDEG for i in range(1, 7)], m.time_stamp

    def gripper_ctrl(self):
        m = self.piper.GetArmGripperCtrl()
        return m.gripper_ctrl.grippers_angle / 1e6, m.time_stamp

    def end_pose(self):
        m = self.piper.GetArmEndPoseMsgs()
        e = m.end_pose
        pose = [e.X_axis / 1e6, e.Y_axis / 1e6, e.Z_axis / 1e6] + [
            math.radians(v / 1000) for v in (e.RX_axis, e.RY_axis, e.RZ_axis)]
        return pose, m.time_stamp


def fresh(sample, timeout, now=None):
    """True when a getter result exists and its CAN stamp is within `timeout`."""
    now = time.time() if now is None else now
    return sample is not None and 0 <= now - sample[-1] <= timeout
