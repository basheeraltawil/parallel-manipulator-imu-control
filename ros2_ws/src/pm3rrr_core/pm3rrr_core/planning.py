"""Singularity-aware path checking and via-point planning.

A straight line between two good poses can still pass through a Type-II
(parallel) singularity, where the platform loses stiffness and the motors
can no longer control it (docs/learning/06_jacobian_singularities.md).
Industrial controllers therefore *check* every planned path and, if needed,
*re-route* it. This module implements the simplest robust version:

  check_segment  -> worst condition number along a straight segment
  plan           -> straight line if safe, else the best single via-point found by
                    a grid search in (x, y, phi) around the midpoint
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import List

import numpy as np

from .kinematics import Pm3rrrKinematics


@dataclass
class SegmentCheck:
    ok: bool
    worst_kappa: float
    reachable: bool
    s_worst: float          # where along the segment (0..1) the worst value occurs


def check_segment(kin: Pm3rrrKinematics, a, b, kappa_max: float = 15.0, n: int = 60) -> SegmentCheck:
    a, b = np.asarray(a, float), np.asarray(b, float)
    worst, s_w = 0.0, 0.0
    for s in np.linspace(0.0, 1.0, n):
        x = a + s * (b - a)
        ik = kin.ik(x)
        if not ik.ok:
            return SegmentCheck(False, np.inf, False, s)
        k = kin.singularity_measures(x, ik)["kappa"]
        if k > worst:
            worst, s_w = k, s
    return SegmentCheck(worst <= kappa_max, worst, True, s_w)


def plan(kin: Pm3rrrKinematics, a, b, kappa_max: float = 15.0, search_radius: float = 0.025,
         grid: int = 11, phi_offsets=np.radians([0.0, -6.0, 6.0, -12.0, 12.0])) -> List[np.ndarray]:
    """Return a list of poses [a, (via), b] whose straight segments are all safe.

    Raises ``RuntimeError`` if neither the direct path nor any single via-point works
    (then the operator must split the motion or change the working mode).
    """
    a, b = np.asarray(a, float), np.asarray(b, float)
    if not same_assembly_mode(kin, a, b):
        raise RuntimeError("goal is in another assembly mode (sign of det Jx differs): "
                           "it cannot be reached without crossing a parallel singularity")
    if check_segment(kin, a, b, kappa_max).ok:
        return [a, b]
    mid = 0.5 * (a + b)
    best, best_val = None, np.inf
    offs = np.linspace(-search_radius, search_radius, grid)
    cands = [mid + np.array([dx, dy, dp]) for dp in phi_offsets for dx in offs for dy in offs]
    for c in cands:
        if same_assembly_mode(kin, a, c):
            ik = kin.ik(c)
            if not ik.ok or kin.singularity_measures(c, ik)["kappa"] > kappa_max:
                continue
            s1 = check_segment(kin, a, c, kappa_max, 30)
            if not s1.ok:
                continue
            s2 = check_segment(kin, c, b, kappa_max, 30)
            if not s2.ok:
                continue
            val = max(s1.worst_kappa, s2.worst_kappa) + 50.0 * np.linalg.norm(c[:2] - mid[:2])
            if val < best_val:
                best, best_val = c, val
    if best is None:
        raise RuntimeError("no safe single via-point found")
    return [a, best, b]


def assembly_sign(kin: Pm3rrrKinematics, x) -> float:
    """Sign of det(Jx): poses with different signs are separated by a Type-II singularity."""
    return float(np.sign(np.linalg.det(kin.jacobians(x)[0])))


def same_assembly_mode(kin: Pm3rrrKinematics, a, b) -> bool:
    return assembly_sign(kin, a) == assembly_sign(kin, b)


def guard_trajectory(kin: Pm3rrrKinematics, poses, kappa_max: float = 15.0):
    """Singularity guard used online: return (index of last safe sample, reason or None).

    The motion must stop BEFORE kappa exceeds the limit, the IK fails, or the
    assembly mode would change. This is what an industrial controller does
    ("singularity protection" / "path stopped before singularity").
    """
    s0 = assembly_sign(kin, poses[0])
    for k, x in enumerate(poses):
        ik = kin.ik(x)
        if not ik.ok:
            return k - 1, f"pose unreachable ({ik.message})"
        if kin.singularity_measures(x, ik)["kappa"] > kappa_max:
            return k - 1, f"condition number above {kappa_max:g} (near a parallel singularity)"
        if assembly_sign(kin, x) != s0:
            return k - 1, "assembly mode would change"
    return len(poses) - 1, None
