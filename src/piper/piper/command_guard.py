"""Command validation/watchdog independent of ROS and the SDK."""
import math


class CommandGuard:
    def __init__(self, gripper=False, timeout=0.25):
        self.names = [f'joint{i}' for i in range(1, 7)] + (['joint7'] if gripper else [])
        self.timeout = timeout
        self.feedback = None
        self.feedback_at = self.ready_at = self.teach_at = -math.inf
        self.ready = False
        self.teaching = False
        self.synchronized = False
        self.active = False
        self.stopped = False
        self.command_at = -math.inf

    def set_feedback(self, names, positions, stamp, wall_now, mono_now):
        if (len(names) != len(positions) or len(set(names)) != len(names)
                or not 0 <= wall_now - stamp <= self.timeout):
            return
        mapping = dict(zip(names, positions))
        if not all(n in mapping and math.isfinite(mapping[n]) for n in self.names):
            return
        self.feedback = [mapping[n] for n in self.names]
        # Preserve source age even if an old message is replayed at high ROS rate.
        self.feedback_at = mono_now - (wall_now - stamp)

    def set_teaching(self, teaching, now):
        if teaching != self.teaching:
            self.synchronized = False
            self.active = False
        self.teaching = teaching
        self.teach_at = now

    def healthy(self, now):
        return (self.ready and self.feedback is not None
                and now - self.ready_at <= self.timeout
                and now - self.feedback_at <= self.timeout
                and now - self.teach_at <= self.timeout)

    def accept(self, names, positions, stamp, wall_now, now):
        if (self.stopped or self.teaching or not self.healthy(now)
                or not 0 <= wall_now - stamp <= self.timeout
                or len(names) != len(positions) or len(set(names)) != len(names)):
            return None
        mapping = dict(zip(names, positions))
        if not all(n in mapping and math.isfinite(mapping[n]) for n in self.names):
            return None
        values = [mapping[n] for n in self.names]
        if not self.synchronized:
            # Startup/teach exit may only acquire the pose at which the arm already is.
            limits = [0.02] * 6 + ([0.002] if len(values) == 7 else [])
            if any(abs(a-b) > tol for a,b,tol in zip(values, self.feedback, limits)):
                return None
            self.synchronized = True
        self.command_at = now
        self.active = True
        return values

    def expired(self, now):
        return (self.active and not self.stopped and not self.teaching
                and (not self.healthy(now) or now - self.command_at > self.timeout))
