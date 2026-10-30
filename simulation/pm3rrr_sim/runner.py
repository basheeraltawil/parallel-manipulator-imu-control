"""Closed-loop runner shared by the scenarios: controller at 50 Hz -> servos -> physics."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Callable, List, Optional

import numpy as np

from pm3rrr_core.params import ServoSpec
from pm3rrr_core.servo import ServoModel

# Upgrade path used in the docs: encoder-feedback "smart" servo (e.g. Dynamixel XL430 class)
SMART_SERVO = ServoSpec(stall_torque=1.4, no_load_speed=6.0, resolution=np.radians(0.088),
                        deadband=np.radians(0.05), time_constant=0.02)


def arduino_map(x, in_min, in_max, out_min, out_max):
    """Exact Arduino ``map()``: integer (long) arithmetic, truncation toward zero."""
    num = (int(x) - in_min) * (out_max - out_min)
    den = in_max - in_min
    q = abs(num) // abs(den) * (1 if num * den >= 0 else -1)
    return q + out_min


def quantise_servo(q, mode: str = "deg"):
    """What the firmware can actually send: Servo.write(int deg) or writeMicroseconds(int us)."""
    deg = 90.0 + np.degrees(np.asarray(q, float))
    if mode == "deg":
        deg = np.round(deg)
    elif mode == "us":                       # 1000..2000 us over 0..180 deg -> 0.18 deg per us
        deg = np.round(deg / 0.18) * 0.18
    elif mode == "smart":                    # 12-bit encoder smart servo: 360/4096 = 0.088 deg
        deg = np.round(deg / 0.088) * 0.088
    return np.radians(deg - 90.0)


@dataclass
class Log:
    t: List[float] = field(default_factory=list)
    target: List[np.ndarray] = field(default_factory=list)
    actual: List[np.ndarray] = field(default_factory=list)
    q_cmd: List[np.ndarray] = field(default_factory=list)
    tau: List[np.ndarray] = field(default_factory=list)
    kappa: List[float] = field(default_factory=list)
    extra: dict = field(default_factory=dict)

    def arrays(self):
        return {k: np.asarray(v) for k, v in self.__dict__.items() if k != "extra"}


def run(sim, target_fn: Callable[[float], np.ndarray], duration: float, dt_ctrl: float = 0.02,
        servo_mode: str = "deg", frame_every: int = 2, on_frame: Optional[Callable] = None,
        log: Optional[Log] = None, dt_servo: float = 0.002):
    """Run the controller loop. ``target_fn(t)`` returns the desired platform pose.

    Each 50 Hz cycle: IK -> joint command -> quantisation (``servo_mode``:
    "deg" = Servo.write, "us" = writeMicroseconds, "ideal" = none). The
    command then goes through the behavioural hobby-servo model (dead band,
    speed limit, first-order lag; pm3rrr_core.servo) whose output drives the
    physics actuator. Unreachable targets are rejected and the last valid
    command is held (exactly what the firmware does).
    """
    kin = sim.kin
    log = log or Log()
    q_last = sim.motor_positions()
    servo = None
    if servo_mode != "ideal":
        spec = SMART_SERVO if servo_mode == "smart" else kin.p.servo
        servo = ServoModel(spec, n=3, use_microseconds=(servo_mode == "us"), q0=q_last)
    n_sub = max(1, int(round(dt_ctrl / dt_servo)))
    frames = []
    n = int(round(duration / dt_ctrl))
    for k in range(n + 1):
        t = k * dt_ctrl
        x_des = np.asarray(target_fn(t), float)
        ik = kin.ik(x_des)
        if ik.ok:
            q_last = kin.motor_to_joint(ik.theta)
        q_cmd = quantise_servo(q_last, servo_mode)
        for _ in range(n_sub):
            sim.command(servo.step(q_cmd, dt_ctrl / n_sub) if servo else q_cmd)
            sim.step(dt_ctrl / n_sub)
        log.t.append(t)
        log.target.append(x_des)
        log.actual.append(sim.platform_pose())
        log.q_cmd.append(q_cmd)
        log.tau.append(sim.motor_torques())
        log.kappa.append(kin.singularity_measures(x_des, ik)["kappa"] if ik.reachable else np.inf)
        if on_frame is not None and k % frame_every == 0:
            frames.append(on_frame(k, t, log))
    return log, frames
