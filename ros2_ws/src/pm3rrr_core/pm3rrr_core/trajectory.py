"""Trajectory generation (see docs/learning/09_trajectory_generation.md).

Two layers, as in any industrial controller:

1. *Time scaling* s(t) in [0, 1]: how fast we move along a path
   (cubic, quintic, minimum-jerk, trapezoidal/LSPB).
2. *Path* x(s): where we go (straight line, circular arc, figure-eight,
   polyline through waypoints).

``sample`` combines both into time-stamped pose / velocity / acceleration
arrays which are then converted to joint space with the inverse kinematics.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, List, Sequence, Tuple

import numpy as np

# ----------------------------------------------------------------- scalings


def cubic(tau: np.ndarray):
    """Cubic polynomial s = 3t^2 - 2t^3 (zero start/end velocity)."""
    tau = np.clip(tau, 0.0, 1.0)
    return 3 * tau**2 - 2 * tau**3, 6 * tau - 6 * tau**2, 6 - 12 * tau


def quintic(tau: np.ndarray):
    """Quintic s = 10t^3 - 15t^4 + 6t^5 (zero start/end vel. AND acc.) = minimum jerk."""
    tau = np.clip(tau, 0.0, 1.0)
    s = 10 * tau**3 - 15 * tau**4 + 6 * tau**5
    ds = 30 * tau**2 - 60 * tau**3 + 30 * tau**4
    dds = 60 * tau - 180 * tau**2 + 120 * tau**3
    return s, ds, dds


minimum_jerk = quintic


def trapezoidal(tau: np.ndarray, accel_fraction: float = 0.25):
    """Linear segment with parabolic blends (LSPB / trapezoidal velocity).

    ``accel_fraction`` = share of the total time spent accelerating (0 < ta <= 0.5).
    """
    ta = float(np.clip(accel_fraction, 1e-6, 0.5))
    tau = np.clip(np.asarray(tau, dtype=float), 0.0, 1.0)
    v = 1.0 / (1.0 - ta)            # cruise speed in normalised units
    a = v / ta
    s = np.where(tau < ta, 0.5 * a * tau**2,
                 np.where(tau <= 1 - ta, v * (tau - ta / 2.0),
                          1.0 - 0.5 * a * (1.0 - tau) ** 2))
    ds = np.where(tau < ta, a * tau, np.where(tau <= 1 - ta, v, a * (1.0 - tau)))
    dds = np.where(tau < ta, a, np.where(tau <= 1 - ta, 0.0, -a))
    return s, ds, dds


SCALINGS = {"cubic": cubic, "quintic": quintic, "minimum_jerk": quintic,
            "trapezoidal": trapezoidal}


def min_duration(distance: float, vmax: float, amax: float, profile: str = "quintic") -> float:
    """Shortest duration respecting |v|<=vmax and |a|<=amax for a given profile."""
    distance = abs(distance)
    if distance == 0:
        return 0.0
    peak = {"cubic": (1.5, 6.0), "quintic": (1.875, 5.7735), "minimum_jerk": (1.875, 5.7735),
            "trapezoidal": (1.0 / 0.75, 1.0 / (0.75 * 0.25))}[profile]
    return max(peak[0] * distance / vmax, np.sqrt(peak[1] * distance / amax))

# -------------------------------------------------------------------- paths


Path = Callable[[np.ndarray], Tuple[np.ndarray, np.ndarray, np.ndarray]]
"""A path maps s (N,) -> (x (N,3), dx/ds (N,3), d2x/ds2 (N,3))."""


def line(start: Sequence[float], goal: Sequence[float]) -> Path:
    a, b = np.asarray(start, float), np.asarray(goal, float)

    def f(s):
        s = np.asarray(s, float)[:, None]
        return a + s * (b - a), np.broadcast_to(b - a, (len(s), 3)).copy(), np.zeros((len(s), 3))
    return f


def circle(center: Sequence[float], radius: float, phi: float = 0.0,
           turns: float = 1.0, start_angle: float = 0.0) -> Path:
    cx, cy = center

    def f(s):
        s = np.asarray(s, float)
        w = 2 * np.pi * turns
        ang = start_angle + w * s
        x = np.column_stack((cx + radius * np.cos(ang), cy + radius * np.sin(ang), np.full_like(s, phi)))
        dx = np.column_stack((-radius * w * np.sin(ang), radius * w * np.cos(ang), np.zeros_like(s)))
        ddx = np.column_stack((-radius * w**2 * np.cos(ang), -radius * w**2 * np.sin(ang), np.zeros_like(s)))
        return x, dx, ddx
    return f


def lemniscate(center: Sequence[float], a: float, phi_amp: float = 0.0) -> Path:
    """Figure-eight (Gerono lemniscate) with an optional orientation wobble."""
    cx, cy = center

    def f(s):
        s = np.asarray(s, float)
        w = 2 * np.pi
        t = w * s
        x = np.column_stack((cx + a * np.sin(t), cy + a * np.sin(t) * np.cos(t), phi_amp * np.sin(t)))
        dx = np.column_stack((a * w * np.cos(t), a * w * np.cos(2 * t), phi_amp * w * np.cos(t)))
        ddx = np.column_stack((-a * w**2 * np.sin(t), -2 * a * w**2 * np.sin(2 * t),
                               -phi_amp * w**2 * np.sin(t)))
        return x, dx, ddx
    return f


# ----------------------------------------------------------------- sampling
@dataclass
class Trajectory:
    t: np.ndarray       # (N,)
    x: np.ndarray       # (N,3) pose
    xd: np.ndarray      # (N,3) velocity
    xdd: np.ndarray     # (N,3) acceleration

    def __len__(self):
        return len(self.t)

    @staticmethod
    def concatenate(parts: List["Trajectory"]) -> "Trajectory":
        out, t0 = [], 0.0
        for k, p in enumerate(parts):
            sl = slice(1 if k else 0, None)
            out.append((p.t[sl] + t0, p.x[sl], p.xd[sl], p.xdd[sl]))
            t0 += p.t[-1]
        return Trajectory(*[np.concatenate([o[i] for o in out]) for i in range(4)])


def sample(path: Path, duration: float, dt: float = 0.01, profile: str = "quintic",
           **kw) -> Trajectory:
    """Time-parameterise ``path`` with a scaling law and sample it every ``dt``."""
    n = max(2, int(round(duration / dt)) + 1)
    t = np.linspace(0.0, duration, n)
    s, ds, dds = SCALINGS[profile](t / duration, **kw)
    x, dx, ddx = path(s)
    sd = (ds / duration)[:, None]
    sdd = (dds / duration**2)[:, None]
    return Trajectory(t, x, dx * sd, ddx * sd**2 + dx * sdd)


def point_to_point(waypoints: Sequence[Sequence[float]], vmax: float = 0.05, wmax: float = 1.0,
                   amax: float = 0.3, dt: float = 0.01, profile: str = "quintic",
                   dwell: float = 0.0) -> Trajectory:
    """Rest-to-rest motion through a list of poses (pick & place style)."""
    parts = []
    for a, b in zip(waypoints[:-1], waypoints[1:]):
        a, b = np.asarray(a, float), np.asarray(b, float)
        lin = np.linalg.norm(b[:2] - a[:2])
        T = max(min_duration(lin, vmax, amax, profile),
                min_duration(abs(b[2] - a[2]), wmax, wmax * 6, profile), 0.2)
        parts.append(sample(line(a, b), T, dt, profile))
        if dwell > 0:
            parts.append(sample(line(b, b), dwell, dt, "cubic"))
    return Trajectory.concatenate(parts)


def to_joint_space(kin, traj: Trajectory):
    """Map a task-space trajectory to joint space (positions, velocities, accelerations).

    Velocities use the inverse Jacobian, accelerations a central difference of
    the velocities (robust and accurate enough for checking servo limits).
    Returns q (N,3) URDF joint values, qd, qdd and the per-sample IK success flags.
    """
    N = len(traj)
    q, qd, ok = np.zeros((N, 3)), np.zeros((N, 3)), np.zeros(N, bool)
    for k in range(N):
        ik = kin.ik(traj.x[k])
        ok[k] = ik.ok
        q[k] = kin.motor_to_joint(ik.theta)
        if ik.reachable:
            qd[k] = kin.inverse_jacobian(traj.x[k], ik) @ traj.xd[k] * np.asarray(kin.p.servo_direction)
    qdd = np.gradient(qd, traj.t, axis=0) if N > 2 else np.zeros_like(qd)
    return q, qd, qdd, ok
