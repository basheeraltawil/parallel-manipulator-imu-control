"""IMU modelling, sensor fusion and tilt-to-motion mapping.

The original project used an MPU6050 (GY-521) and mapped *raw* gyro counts
straight to platform displacement with Arduino ``map()``. That has three
problems this module fixes and the docs explain (docs/learning/10_imu_sensor_fusion.md):

1. a gyro measures *rate*, not angle -> the platform jittered and never held still;
2. raw counts include bias and noise -> drift and shaking;
3. integer ``map()`` loses resolution and has no dead band or limits.

Here: a realistic MPU6050 model, a complementary filter (roll/pitch),
and a TiltTeleop mapper with dead band, expo curve, rate limiting and
workspace clamping.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional

import numpy as np

G = 9.80665


@dataclass
class Mpu6050Model:
    """Simplified MPU6050: +-2 g accel (16384 LSB/g), +-250 dps gyro (131 LSB/dps)."""

    accel_noise: float = 0.004 * G          # m/s^2 rms (datasheet 400 ug/sqrt(Hz) @ ~100 Hz BW)
    gyro_noise: float = np.radians(0.05)    # rad/s rms
    gyro_bias: np.ndarray = field(default_factory=lambda: np.radians([0.8, -0.5, 0.3]))
    accel_bias: np.ndarray = field(default_factory=lambda: np.array([0.05, -0.03, 0.08]))
    seed: Optional[int] = 0

    def __post_init__(self):
        self.rng = np.random.default_rng(self.seed)

    def measure(self, roll: float, pitch: float, omega: np.ndarray, lin_acc=np.zeros(3)):
        """Return (accel [m/s^2], gyro [rad/s]) in the sensor frame for a given attitude."""
        g_body = G * np.array([-np.sin(pitch), np.sin(roll) * np.cos(pitch),
                               np.cos(roll) * np.cos(pitch)])
        acc = g_body + lin_acc + self.accel_bias + self.rng.normal(0, self.accel_noise, 3)
        gyr = np.asarray(omega) + self.gyro_bias + self.rng.normal(0, self.gyro_noise, 3)
        return acc, gyr

    @staticmethod
    def to_raw(acc, gyr):
        """Quantise to the int16 register values the Arduino actually reads."""
        a = np.clip(np.round(np.asarray(acc) / G * 16384), -32768, 32767).astype(int)
        g = np.clip(np.round(np.degrees(gyr) * 131), -32768, 32767).astype(int)
        return a, g


def accel_tilt(acc: np.ndarray):
    """Roll/pitch from gravity alone (valid when the sensor is not accelerating)."""
    ax, ay, az = acc
    roll = np.arctan2(ay, az)
    pitch = np.arctan2(-ax, np.hypot(ay, az))
    return roll, pitch


class ComplementaryFilter:
    """angle = a*(angle + gyro*dt) + (1-a)*accel_angle  (the classic 2-line fusion)."""

    def __init__(self, tau: float = 0.5, dt: float = 0.01):
        self.alpha = tau / (tau + dt)
        self.roll = 0.0
        self.pitch = 0.0
        self.bias = np.zeros(3)
        self._init = False

    def calibrate(self, gyro_samples: np.ndarray):
        """Gyro bias = mean of samples taken while the sensor is still (do this at power-up!)."""
        self.bias = np.mean(np.asarray(gyro_samples), axis=0)

    def update(self, acc, gyr, dt):
        r_acc, p_acc = accel_tilt(acc)
        if not self._init:
            self.roll, self.pitch, self._init = r_acc, p_acc, True
            return self.roll, self.pitch
        g = np.asarray(gyr) - self.bias
        self.roll = self.alpha * (self.roll + g[0] * dt) + (1 - self.alpha) * r_acc
        self.pitch = self.alpha * (self.pitch + g[1] * dt) + (1 - self.alpha) * p_acc
        return self.roll, self.pitch


class TiltTeleop:
    """Maps hand-held IMU tilt to a platform target pose (the project's original idea).

    pitch -> x, roll -> y, optional yaw-rate -> phi. Shaping chain:
    dead band -> normalise -> expo curve -> scale to workspace -> rate limit -> clamp.
    """

    def __init__(self, max_tilt=np.radians(30), deadband=np.radians(3), expo=0.4,
                 reach=0.018, max_speed=0.06, home=(0.0, 0.0, 0.0), kin=None):
        self.max_tilt, self.deadband, self.expo = max_tilt, deadband, expo
        self.reach, self.max_speed = reach, max_speed
        self.home = np.asarray(home, float)
        self.target = self.home.copy()
        self.kin = kin

    def _shape(self, angle):
        a = np.sign(angle) * max(0.0, abs(angle) - self.deadband)
        n = np.clip(a / (self.max_tilt - self.deadband), -1.0, 1.0)
        return (1 - self.expo) * n + self.expo * n**3

    def update(self, roll, pitch, dt, phi_cmd: float = 0.0):
        goal = self.home + np.array([self.reach * self._shape(pitch),
                                     self.reach * self._shape(roll), phi_cmd])
        step = goal - self.target
        n = np.linalg.norm(step[:2])
        lim = self.max_speed * dt
        if n > lim:
            step[:2] *= lim / n
        cand = self.target + step
        if self.kin is None or self.kin.ik(cand).ok:   # never command an unreachable pose
            self.target = cand
        return self.target.copy()
