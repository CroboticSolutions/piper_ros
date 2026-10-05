#!/usr/bin/env python3
"""Cyclic PiPER CAN emulator for vcan tests. Never point it at a real arm's bus."""
import argparse
import struct
import threading
import time

import can


class FakePiper:
    def __init__(self, channel, gripper=True, rate=200.0):
        if not channel.startswith('vcan'):
            raise ValueError('FakePiper only runs on vcan* channels')
        self.bus = can.Bus(interface='socketcan', channel=channel)
        self.gripper = gripper
        self.period = 1.0 / rate
        self.joints_mdeg = [1000, -2000, -30000, 40000, -50000, 60000]
        self.gripper_um = 40000
        self.enabled = [False] * 7
        self.ctrl_mode = 0x01
        self.estopped = False
        self.commands = []
        self.stop = threading.Event()
        self.threads = [threading.Thread(target=self._rx, daemon=True),
                        threading.Thread(target=self._tx, daemon=True)]

    def start(self):
        for t in self.threads:
            t.start()
        return self

    def close(self):
        self.stop.set()
        for t in self.threads:
            t.join(1.0)
        self.bus.shutdown()

    def _rx(self):
        while not self.stop.is_set():
            m = self.bus.recv(0.05)
            if m is None:
                continue
            d = bytes(m.data)
            self.commands.append((m.arbitration_id, d.hex()))
            if m.arbitration_id == 0x471 and d[0] in (7, 0xFF):
                self.enabled = [d[1] == 0x02] * 7
            elif m.arbitration_id == 0x150 and d[0] == 0x01:
                self.estopped = True
            elif m.arbitration_id == 0x159:
                self.enabled[6] = d[6] in (0x01, 0x03)

    def _send(self, can_id, data):
        self.bus.send(can.Message(arbitration_id=can_id, data=data, is_extended_id=False))

    def _tx(self):
        while not self.stop.is_set():
            j = self.joints_mdeg
            self._send(0x2A1, bytes([self.ctrl_mode, 0x07 if self.estopped else 0x00,
                                     0x01, 0, 0, 0, 0, 0]))
            self._send(0x2A2, struct.pack('>ii', 123456, -234567))
            self._send(0x2A3, struct.pack('>ii', 345678, 10000))
            self._send(0x2A4, struct.pack('>ii', -20000, 30000))
            for k, can_id in enumerate((0x2A5, 0x2A6, 0x2A7)):
                self._send(can_id, struct.pack('>ii', j[2 * k], j[2 * k + 1]))
            for i in range(6):
                self._send(0x251 + i, struct.pack('>hhi', 100 * (i + 1), 50 * (i + 1), 0))
                foc = 0x40 if self.enabled[i] else 0x00
                self._send(0x261 + i, struct.pack('>HhbB', 240, 30, 25, foc) + b'\0\0')
            if self.gripper:
                foc = 0x40 if self.enabled[6] else 0x00
                self._send(0x2A8, struct.pack('>ihBB', self.gripper_um, 1500, foc, 0))
            time.sleep(self.period)


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--channel', default='vcan0')
    ap.add_argument('--no-gripper', action='store_true')
    a = ap.parse_args()
    f = FakePiper(a.channel, not a.no_gripper).start()
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        f.close()
