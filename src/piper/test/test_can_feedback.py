import math
import struct
from piper.can_feedback import CanFeedback
from piper.units import to_radians, to_millidegrees
from piper.command_guard import CommandGuard


def populated(gripper=False):
    f = CanFeedback(gripper)
    for i in f.required:
        data = bytes(8)
        if 0x261 <= i <= 0x266:
            data = struct.pack('>HhbBH', 240, 35, 30, 0x40, 0)
        if i == 0x2A1:
            data = bytes([1, 0, 1, 0, 0, 0, 0, 0])
        if i == 0x2A8:
            data = struct.pack('>ihBB', 40000, 1000, 0xC0, 0)
        f.update(i, data, 10.0)
    return f


def test_known_angles_and_quantization():
    for degrees in [-180, -90, -30, 0, 30, 90, 180]:
        assert to_millidegrees(math.radians(degrees)) == degrees * 1000
        assert math.isclose(to_radians(degrees * 1000), math.radians(degrees), abs_tol=1e-15)
    for r in [0.12345, -1.23456, 2.98765]:
        assert abs(to_radians(to_millidegrees(r))-r) <= math.radians(0.0005)


def test_each_joint_pair_must_be_fresh_even_when_other_traffic_continues():
    f = populated()
    for i in f.required - {0x2A6}:
        f.update(i, f.frames[i][0], 10.4)
    assert f.snapshot(10.4) is None
    f.update(0x2A6, struct.pack('>ii', -90000, 45000), 10.4)
    s = f.snapshot(10.4)
    assert s['ready']
    assert s['positions'][2:4] == [to_radians(-90000), to_radians(45000)]


def test_commands_cannot_fake_measured_position_and_partial_startup_is_invalid():
    f = CanFeedback()
    f.update(0x155, struct.pack('>ii', 90000, 90000), 10.0)
    assert not f.frames
    assert f.snapshot(10.0) is None


def test_fault_disable_and_gripper_mapping():
    f = populated(True)
    assert f.snapshot(10.1)['positions'][-2:] == [0.02, -0.02]
    f.update(0x263, struct.pack('>HhbBH', 240, 35, 30, 0x44, 0), 10.0)
    assert not f.snapshot(10.1)['ready']
    f.update(0x263, struct.pack('>HhbBH', 240, 35, 30, 0, 0), 10.0)
    assert not f.snapshot(10.1)['ready']
    f.update(0x2A1, bytes([2, 0x0B, 1, 0, 0, 0, 0, 0]), 10.0)
    assert f.snapshot(10.1)['ready']
    assert f.snapshot(10.1)['teaching']


def healthy_guard():
    g = CommandGuard()
    g.ready = True
    g.ready_at = 10.0
    g.set_teaching(False, 10.0)
    g.set_feedback(g.names, [0.5]*6, 100, 100, 10)
    return g


def test_startup_cannot_jump_from_measured_pose_to_yaml_zero():
    g = healthy_guard()
    assert g.accept(g.names, [0]*6, 100, 100, 10) is None
    assert g.accept(g.names, [0.5]*6, 100, 100, 10) == [0.5]*6
    assert g.accept(g.names[:-1], [0.5]*5, 100, 100, 10) is None
    assert g.accept(g.names, [float('nan')]*6, 100, 100, 10) is None
    assert g.expired(10.3)


def test_teach_suppresses_commands_and_exit_requires_pose_sync():
    g = healthy_guard()
    g.accept(g.names, [0.5]*6, 100, 100, 10)
    g.set_teaching(True, 10.0)
    assert g.accept(g.names, [0.5]*6, 100, 100, 10) is None
    assert not g.expired(11)
    g.set_feedback(g.names, [0.8]*6, 100, 100, 10)
    g.set_teaching(False, 10)
    assert g.accept(g.names, [0.5]*6, 100, 100, 10) is None
    assert g.accept(g.names, [0.8]*6, 100, 100, 10) == [0.8]*6


def test_replayed_old_measurement_or_command_cannot_extend_watchdog():
    g = healthy_guard()
    assert g.accept(g.names, [0.5]*6, 99, 100, 10) is None
    g.accept(g.names, [0.5]*6, 100, 100, 10)
    g.set_feedback(g.names, [0.5]*6, 100, 100.3, 10.3)
    assert g.expired(10.3)


def test_ended_drag_record_is_not_active_teaching():
    f = populated()
    for mode, teach, expected in [(2, 1, True), (2, 2, False), (2, 0, True), (6, 2, True), (1, 2, False)]:
        f.update(0x2A1, bytes([mode, 0, 1, teach, 0, 0, 0, 0]), 10.)
        snapshot = f.snapshot(10.1)
        assert snapshot['teaching'] == expected
        assert snapshot['ready']
    # End-record is not permission to bypass disabled motor or arm faults.
    f.update(0x2A1, bytes([2, 0, 1, 2, 0, 0, 0, 0]), 10.)
    f.update(0x261, struct.pack('>HhbBH', 240, 35, 30, 0, 0), 10.)
    assert not f.snapshot(10.1)['ready']
