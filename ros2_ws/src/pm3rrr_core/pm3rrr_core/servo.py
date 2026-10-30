"""Behavioural model of a hobby RC servo (MG996R / MG90S).

A hobby servo is a closed-loop position actuator: we send a pulse width
(1000-2000 us ~ 0..180 deg) and its internal controller moves the horn.
For simulation we model what matters for accuracy:

* command quantisation (``Servo.write(int deg)`` -> 1 deg steps!)
* dead band of the internal comparator
* speed saturation (no-load speed) and a first-order lag
"""

from __future__ import annotations

import numpy as np

from .params import ServoSpec


class ServoModel:
    def __init__(self, spec: ServoSpec = ServoSpec(), n: int = 3, use_microseconds: bool = False,
                 q0=None):
        self.s = spec
        self.q = np.zeros(n) if q0 is None else np.asarray(q0, float).copy()
        # writeMicroseconds(): 1000 us / 180 deg -> ~0.18 deg per step
        self.quantum = np.radians(180.0 / 1000.0) if use_microseconds else spec.resolution

    def quantise(self, q_cmd):
        return np.round(np.asarray(q_cmd) / self.quantum) * self.quantum

    def step(self, q_cmd, dt: float):
        err = self.quantise(q_cmd) - self.q
        err = np.where(np.abs(err) < self.s.deadband, 0.0, err)
        v = np.clip(err / self.s.time_constant, -self.s.no_load_speed, self.s.no_load_speed)
        self.q = self.q + v * dt
        return self.q.copy()
