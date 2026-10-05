"""pyAgxArm and piper_sdk backends must emit the same CAN frames and decode the same feedback.

Needs a vcan interface (`ip link add dev vcanpt type vcan && ip link set vcanpt up`);
skipped otherwise. Never runs against a real bus.
"""
import math
import os
import subprocess
import sys
import threading
import time

import can
import pytest

sys.path.insert(0, os.path.dirname(__file__))
from fake_piper_can import FakePiper  # noqa: E402
from piper.sdk_backend import create_backend  # noqa: E402

CH = os.environ.get('PIPER_TEST_VCAN', 'vcanpt')
J = [0.1, -0.2, -0.5, 1.0, -1.2217, 2.0]


def _vcan_up():
    out = subprocess.run(['ip', '-br', 'link', 'show', CH], capture_output=True, text=True)
    return out.returncode == 0 and 'vcan' in subprocess.run(
        ['ip', '-d', 'link', 'show', CH], capture_output=True, text=True).stdout


pytestmark = pytest.mark.skipif(not _vcan_up(), reason=f'{CH} vcan not available')


class Recorder:
    def __init__(self):
        self.bus = can.Bus(interface='socketcan', channel=CH)
        self.frames = []
        self.stop = threading.Event()
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def _run(self):
        while not self.stop.is_set():
            m = self.bus.recv(0.02)
            # Host commands only; FakePiper feedback is 0x251-0x266 and 0x2A1-0x2A8.
            if m is not None and not 0x250 <= m.arbitration_id <= 0x2AF:
                self.frames.append((m.arbitration_id, bytes(m.data).hex()))

    def take(self, fn):
        time.sleep(0.05)
        self.frames.clear()
        fn()
        time.sleep(0.1)
        return list(self.frames)

    def close(self):
        self.stop.set()
        self.thread.join(1)
        self.bus.shutdown()


@pytest.fixture(scope='module')
def setup():
    fake = FakePiper(CH).start()
    rec = Recorder()
    backends = {}
    for name in ('piper_sdk', 'pyagxarm'):
        b = create_backend(name, CH, gripper=True)
        b.connect()
        backends[name] = b
    time.sleep(0.4)
    yield fake, rec, backends
    rec.close()
    fake.close()
    for b in backends.values():
        b.close()


ACTIONS = {
    'enable': lambda b: b.enable(),
    'disable': lambda b: b.disable(),
    'move_j': lambda b: b.move_j(J),  # first command: speed must already be 100 %
    'move_p': lambda b: b.move_p([0.2, 0.0, 0.3, 0.1, 0.2, -0.3], 50),
    'move_j_after_p': lambda b: b.move_j(J),
    'move_j_repeat': lambda b: b.move_j(J),
    'quick_stop': lambda b: b.quick_stop(),
    'gripper': lambda b: b.gripper_command(0.05),
}


@pytest.mark.parametrize('action', list(ACTIONS))
def test_same_command_frames(setup, action):
    _, rec, backends = setup
    frames = {n: rec.take(lambda: ACTIONS[action](b)) for n, b in backends.items()}
    assert frames['pyagxarm'], action
    assert frames['pyagxarm'] == frames['piper_sdk']


def test_mode_frame_throttled_by_pyagxarm(setup):
    # pyAgxArm resends an identical 0x151 at most every 0.1 s; joint frames always go out.
    _, rec, backends = setup
    for name, expected in (('piper_sdk', 3), ('pyagxarm', 1)):
        frames = rec.take(lambda: [backends[name].move_j(J) for _ in range(3)])
        assert [i for i, _ in frames].count(0x151) == expected, name
        assert [i for i, _ in frames].count(0x155) == 3, name


def test_same_feedback(setup):
    _, _, backends = setup
    old, new = backends['piper_sdk'], backends['pyagxarm']
    for getter in ('joints', 'end_pose'):
        a, b = getattr(old, getter)()[0], getattr(new, getter)()[0]
        assert all(math.isclose(x, y, abs_tol=1e-9) for x, y in zip(a, b)), getter
    (va, ea, _), (vb, eb, _) = old.motors(), new.motors()
    assert va == pytest.approx(vb) and ea == pytest.approx(eb)
    assert old.gripper_state()[:2] == pytest.approx(new.gripper_state()[:2])
    assert vars(old.status()) == vars(new.status())
    assert old.motors_enabled() == new.motors_enabled()
    assert old.is_ok() and new.is_ok()


def test_feedback_loss_detected(setup):
    fake, _, backends = setup
    fake.close()
    time.sleep(1.0)
    assert not backends['pyagxarm'].is_ok()
