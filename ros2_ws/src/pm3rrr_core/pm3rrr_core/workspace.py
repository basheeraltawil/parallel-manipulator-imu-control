"""Workspace analysis (see docs/learning/07_workspace_analysis.md).

* constant-orientation workspace: all (x, y) reachable at a fixed phi
* orientation range: for each (x, y), the range of phi that is reachable
* dexterity maps: condition number / Type-II measure over the workspace
* working-mode comparison: the 2^3 = 8 elbow branches
"""

from __future__ import annotations

from dataclasses import dataclass
from itertools import product

import numpy as np

from .kinematics import Pm3rrrKinematics


@dataclass
class WorkspaceMap:
    xs: np.ndarray
    ys: np.ndarray
    reachable: np.ndarray     # (ny, nx) bool: legs close AND servos within travel
    kappa: np.ndarray         # (ny, nx) condition number (nan outside)
    type2: np.ndarray         # (ny, nx) parallel-singularity measure (nan outside)

    @property
    def area(self) -> float:
        dx = self.xs[1] - self.xs[0]
        dy = self.ys[1] - self.ys[0]
        return float(self.reachable.sum() * dx * dy)

    def usable(self, kappa_max: float = 5.0) -> np.ndarray:
        """Reachable AND well conditioned (the region you should actually use)."""
        return self.reachable & (np.nan_to_num(self.kappa, nan=np.inf) <= kappa_max)


def constant_orientation(kin: Pm3rrrKinematics, phi: float = 0.0, half_size: float = 0.06,
                         n: int = 121, modes=None, check_limits: bool = True) -> WorkspaceMap:
    xs = np.linspace(-half_size, half_size, n)
    ys = np.linspace(-half_size, half_size, n)
    R = np.zeros((n, n), bool)
    K = np.full((n, n), np.nan)
    T2 = np.full((n, n), np.nan)
    for j, y in enumerate(ys):
        for i, x in enumerate(xs):
            ik = kin.ik((x, y, phi), modes)
            ok = ik.ok if check_limits else ik.reachable
            if ok:
                R[j, i] = True
                m = kin.singularity_measures((x, y, phi), ik)
                K[j, i] = m["kappa"]
                T2[j, i] = m["type2"]
    return WorkspaceMap(xs, ys, R, K, T2)


def orientation_range(kin: Pm3rrrKinematics, half_size: float = 0.05, n: int = 51,
                      phi_max: float = np.radians(90), n_phi: int = 91, kappa_max: float = 20.0):
    """For each (x, y): the contiguous range of phi (containing 0 if possible) that is usable
    (reachable, servo limits, kappa <= kappa_max, same assembly mode as the home pose)."""
    xs = np.linspace(-half_size, half_size, n)
    ys = np.linspace(-half_size, half_size, n)
    phis = np.linspace(-phi_max, phi_max, n_phi)
    lo = np.full((n, n), np.nan)
    hi = np.full((n, n), np.nan)
    for j, y in enumerate(ys):
        for i, x in enumerate(xs):
            ok = np.array([_usable(kin, (x, y, ph), kappa_max) for ph in phis])
            if not ok.any():
                continue
            k0 = np.argmin(np.abs(phis)) if ok[np.argmin(np.abs(phis))] else int(np.flatnonzero(ok)[0])
            a = b = k0
            while a > 0 and ok[a - 1]:
                a -= 1
            while b < n_phi - 1 and ok[b + 1]:
                b += 1
            lo[j, i], hi[j, i] = phis[a], phis[b]
    return xs, ys, lo, hi


def _usable(kin, pose, kappa_max):
    """Reachable, within servo limits, well conditioned AND in the home assembly mode."""
    ik = kin.ik(pose)
    if not ik.ok:
        return False
    Jx, _ = kin.jacobians(pose, ik)
    if np.linalg.det(Jx) * _HOME_SIGN.setdefault(id(kin), np.sign(np.linalg.det(kin.jacobians((0, 0, 0))[0]))) <= 0:
        return False
    return kin.singularity_measures(pose, ik)["kappa"] <= kappa_max


_HOME_SIGN: dict = {}


def working_modes_summary(kin: Pm3rrrKinematics, phi: float = 0.0, half_size: float = 0.06,
                          n: int = 61, kappa_max: float = 5.0):
    """Area and usable (well-conditioned) area of the 8 working modes, ignoring servo limits."""
    out = []
    for modes in product((1.0, -1.0), repeat=3):
        w = constant_orientation(kin, phi, half_size, n, modes, check_limits=False)
        out.append({"modes": modes, "area_cm2": w.area * 1e4,
                    "usable_cm2": float(w.usable(kappa_max).sum() * (w.xs[1] - w.xs[0]) ** 2 * 1e4)})
    return out


def largest_inscribed_circle(wmap: WorkspaceMap, kappa_max: float = 5.0, center=(0.0, 0.0)) -> float:
    """Radius of the largest circle around ``center`` fully inside the usable workspace.

    This is the number you put in a datasheet: "circular task space of diameter D".
    """
    U = wmap.usable(kappa_max)
    X, Y = np.meshgrid(wmap.xs, wmap.ys)
    d = np.hypot(X - center[0], Y - center[1])
    bad = d[~U]
    return float(bad.min()) if bad.size else float(d.max())
