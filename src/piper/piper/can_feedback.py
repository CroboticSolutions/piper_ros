"""Passive CAN decoder with freshness per frame, not an aggregate SDK timestamp."""
import math
import struct
from piper.units import to_radians


class CanFeedback:
    def __init__(self, gripper=False, timeout=0.25):
        self.gripper = gripper
        self.timeout = timeout
        self.frames = {}
        self.required = ({0x2A1, 0x2A5, 0x2A6, 0x2A7}
                         | set(range(0x251, 0x257)) | set(range(0x261, 0x267)))
        if gripper:
            self.required.add(0x2A8)

    def update(self, can_id, data, stamp):
        if can_id in self.required and len(data) == 8 and math.isfinite(stamp):
            self.frames[can_id] = (bytes(data), stamp)

    def fresh(self, now):
        return all(i in self.frames and 0 <= now - self.frames[i][1] <= self.timeout
                   for i in self.required)

    def snapshot(self, now):
        if not self.fresh(now):
            return None
        status = self.frames[0x2A1][0]
        # Firmware can retain ctrl_mode=2 after the operator exits drag teach.
        # Byte 3 == 2 explicitly reports end-record / exit-drag. Keeping that
        # state blocked prevents the first measured hold from returning to CAN mode.
        teaching = (status[0] == 6 or (status[0] == 2 and status[3] != 2))
        ready = (status[0] in (0, 1, 2, 6)
                 and (status[1] == 0 or (teaching and status[1] in (0x0B, 0x0C, 0x0D)))
                 and status[6:8] == b'\0\0')
        for i in range(0x261, 0x267):
            flags = self.frames[i][0][5]
            ready = ready and not (flags & 0xBF) and (teaching or bool(flags & 0x40))
        positions = [to_radians(v) for i in (0x2A5, 0x2A6, 0x2A7)
                     for v in struct.unpack('>ii', self.frames[i][0])]
        velocities = [struct.unpack('>h', self.frames[i][0][:2])[0] / 1000.0
                      for i in range(0x251, 0x257)]
        # Preserve the installed SDK's current-derived effort convention (not a load cell).
        efforts = [struct.unpack('>h', self.frames[i][0][2:4])[0]
                   * (1.18125 if i <= 0x253 else 0.95844) / 1000.0
                   for i in range(0x251, 0x257)]
        names = [f'joint{i}' for i in range(1, 7)]
        if self.gripper:
            data = self.frames[0x2A8][0]
            stroke = struct.unpack('>i', data[:4])[0] / 1e6
            ready = ready and not (data[6] & 0x3F) and (teaching or bool(data[6] & 0x40))
            names += ['joint7', 'joint8']
            positions += [stroke / 2, -stroke / 2]
            force = struct.unpack('>h', data[4:6])[0] / 1000.0
            efforts += [force / 2, -force / 2]
            # No gripper velocity in this protocol. Its controller uses position only.
            velocities += [0.0, 0.0]
        return dict(names=names, positions=positions, velocities=velocities, efforts=efforts,
                    stamp=min(self.frames[i][1] for i in self.required),
                    ready=bool(ready), teaching=teaching)
