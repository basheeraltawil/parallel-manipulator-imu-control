"""Kinematics of the planar 3-RRR parallel manipulator.

Notation (see docs/learning/04_inverse_kinematics.md):

* pose ``x = (px, py, phi)``: platform centre position and orientation
* ``B_i``  : motor axis of leg i (fixed, world frame)
* ``A_i``  : elbow of leg i
* ``C_i``  : platform joint of leg i, ``C_i = p + R(phi) c_i``
* ``theta_i``: absolute angle of the proximal link (actuated joint)
* ``sigma_i``: working mode (elbow branch), ``theta_i = beta_i + sigma_i * alpha_i``

Every function is pure numpy so it can be used from the analysis scripts,
from the ROS 2 nodes and from the unit tests alike.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional, Sequence

import numpy as np

from .params import DEFAULT_PARAMS, RobotParams

E90 = np.array([[0.0, -1.0], [1.0, 0.0]])  # 90 deg rotation (planar cross product helper)


def rot2(phi: float) -> np.ndarray:
    c, s = np.cos(phi), np.sin(phi)
    return np.array([[c, -s], [s, c]])


def wrap(angle):
    """Wrap angle(s) to (-pi, pi]."""
    return (np.asarray(angle) + np.pi) % (2.0 * np.pi) - np.pi


@dataclass
class IKResult:
    theta: np.ndarray          # (3,) actuated joint angles, world frame [rad]
    elbows: np.ndarray         # (3,2) A_i
    platform_joints: np.ndarray  # (3,2) C_i
    reachable: bool            # every leg can geometrically close
    within_limits: bool        # every servo inside its travel
    message: str = ""

    @property
    def ok(self) -> bool:
        return self.reachable and self.within_limits


class Pm3rrrKinematics:
    """Inverse/forward/velocity kinematics and singularity measures."""

    def __init__(self, params: RobotParams = DEFAULT_PARAMS):
        self.p = params
        self.B = params.base_points()
        self.c = params.platform_points()
        self.modes = np.asarray(params.working_modes, dtype=float)
        # URDF convention: every joint is 0 at the assembled home pose -> store home offsets
        self._home = None
        self._home = self._raw_passive((0.0, 0.0, 0.0))

    # ------------------------------------------------------------------ IK
    def platform_joints(self, pose: Sequence[float]) -> np.ndarray:
        px, py, phi = pose
        return np.array([px, py]) + self.c @ rot2(phi).T

    def ik(self, pose: Sequence[float], modes: Optional[Sequence[float]] = None) -> IKResult:
        """Closed-form inverse kinematics (generalises the report's eq. 1-7).

        For each leg the triangle B_i-A_i-C_i has sides l1, l2 and d_i=|C_i-B_i|.
        beta_i  = atan2(d_y, d_x)                  (direction of the leg)
        alpha_i = acos((l1^2 + d^2 - l2^2) / (2 l1 d))   (law of cosines)
        theta_i = beta_i + sigma_i * alpha_i
        With l1 = l2 = L this is exactly a = d/2, h = sqrt(L^2 - a^2),
        alpha = atan2(h, a) from the original report.
        """
        sig = self.modes if modes is None else np.asarray(modes, dtype=float)
        l1, l2 = self.p.l1, self.p.l2
        C = self.platform_joints(pose)
        d = C - self.B
        dist = np.linalg.norm(d, axis=1)
        beta = np.arctan2(d[:, 1], d[:, 0])
        cos_a = (l1**2 + dist**2 - l2**2) / (2.0 * l1 * np.maximum(dist, 1e-12))
        reachable = bool(np.all(np.abs(cos_a) <= 1.0))
        alpha = np.arccos(np.clip(cos_a, -1.0, 1.0))
        theta = wrap(beta + sig * alpha)
        A = self.B + l1 * np.column_stack((np.cos(theta), np.sin(theta)))
        q = self.motor_to_joint(theta)
        within = bool(np.all(np.abs(q) <= self.p.joint_limit + 1e-9))
        msg = ""
        if not reachable:
            msg = "leg(s) %s cannot reach" % (np.where(np.abs(cos_a) > 1.0)[0] + 1).tolist()
        elif not within:
            msg = "servo limit exceeded on leg(s) %s" % (
                np.where(np.abs(q) > self.p.joint_limit)[0] + 1).tolist()
        return IKResult(theta, A, C, reachable, within, msg)

    # ----------------------------------------------- joint/servo conversions
    def motor_to_joint(self, theta: np.ndarray) -> np.ndarray:
        """World link angle -> URDF motor joint value (0 = servo centre)."""
        return wrap((np.asarray(theta) - np.asarray(self.p.servo_center)) *
                    np.asarray(self.p.servo_direction))

    def joint_to_motor(self, q: np.ndarray) -> np.ndarray:
        return wrap(np.asarray(q) * np.asarray(self.p.servo_direction) +
                    np.asarray(self.p.servo_center))

    def joint_to_servo_deg(self, q: np.ndarray) -> np.ndarray:
        """URDF joint value -> hobby-servo command in degrees (0..180, 90 = centre)."""
        return 90.0 + np.degrees(np.asarray(q))

    def servo_deg_to_joint(self, deg: np.ndarray) -> np.ndarray:
        return np.radians(np.asarray(deg) - 90.0)

    # -------------------------------------------------------- full config
    def passive_angles(self, pose: Sequence[float], theta: np.ndarray, A: np.ndarray):
        """Return (elbow joint angles psi_i, distal-link absolute angles)."""
        C = self.platform_joints(pose)
        u = C - A
        distal = np.arctan2(u[:, 1], u[:, 0])
        psi = wrap(distal - theta)
        return psi, distal

    def _raw_passive(self, pose, ik=None):
        ik = ik or self.ik(pose)
        psi, distal = self.passive_angles(pose, ik.theta, ik.elbows)
        return psi, wrap(pose[2] - distal)

    @property
    def home_offsets(self):
        """(elbow angles, platform-joint angles) at home: the URDF joint-origin yaws."""
        return self._home

    def joint_state(self, pose: Sequence[float], ik: Optional[IKResult] = None,
                    include_loop_closures: bool = False) -> dict:
        """All URDF tree joint values for robot_state_publisher / RViz.

        Tree: base -> motor_i -> elbow_i ; leg1 distal -> platform (leg1_platform_joint).
        Legs 2 and 3 close on the platform through loop-closure joints.
        Convention: every value is 0 at the home pose (offsets live in the URDF origins).
        """
        ik = ik or self.ik(pose)
        q = self.motor_to_joint(ik.theta)
        psi, plat = self._raw_passive(pose, ik)
        psi0, plat0 = self._home
        js = {}
        for i in range(3):
            js[f"leg{i+1}_motor_joint"] = float(q[i])
            js[f"leg{i+1}_elbow_joint"] = float(wrap(psi[i] - psi0[i]))
            if i == 0 or include_loop_closures:
                js[f"leg{i+1}_platform_joint"] = float(wrap(plat[i] - plat0[i]))
        return js

    # ------------------------------------------------------------------ FK
    def fk_all(self, theta: Sequence[float], n_phi: int = 721) -> List[np.ndarray]:
        """All assembly modes (up to 6) for given actuated angles.

        Method (educational, robust): for a trial orientation phi, the
        platform centre p must lie on circles centred at A_i - R(phi) c_i with
        radius l2. Intersect the circles of legs 1 and 2 (two candidates) and
        evaluate the residual of leg 3. Roots of that residual in phi are
        refined with Newton's method.
        """
        theta = np.asarray(theta, dtype=float)
        A = self.B + self.p.l1 * np.column_stack((np.cos(theta), np.sin(theta)))
        l2 = self.p.l2

        def candidates(phi):
            R = rot2(phi)
            K = A - self.c @ R.T                       # circle centres for p
            d = K[1] - K[0]
            dn = np.linalg.norm(d)
            if dn < 1e-12 or dn > 2 * l2:
                return []
            h2 = l2**2 - (dn / 2.0) ** 2
            if h2 < 0:
                return []
            mid = K[0] + d / 2.0
            perp = E90 @ d / dn
            out = []
            for s in (+1.0, -1.0):
                p = mid + s * np.sqrt(h2) * perp
                out.append((p, np.linalg.norm(p - K[2]) - l2))
            return out

        phis = np.linspace(-np.pi, np.pi, n_phi)
        sols = []
        for branch in (0, 1):
            f = []
            for ph in phis:
                c = candidates(ph)
                f.append(c[branch][1] if len(c) == 2 else np.nan)
            f = np.asarray(f)
            for k in range(len(phis) - 1):
                if np.isfinite(f[k]) and np.isfinite(f[k + 1]) and f[k] * f[k + 1] <= 0:
                    p = candidates(phis[k])[branch][0]
                    guess = np.array([p[0], p[1], phis[k]])
                    sol = self.fk(theta, guess)
                    if sol is not None and not any(
                            np.allclose(sol, s, atol=1e-6) or
                            (np.allclose(sol[:2], s[:2], atol=1e-6) and abs(wrap(sol[2] - s[2])) < 1e-6)
                            for s in sols):
                        sols.append(sol)
        return sols

    def fk(self, theta: Sequence[float], guess: Sequence[float], tol: float = 1e-12,
           max_iter: int = 50) -> Optional[np.ndarray]:
        """Newton-Raphson forward kinematics from an initial guess.

        Used in real time (previous pose = excellent guess), it converges in
        2-3 iterations and stays in the current assembly mode.
        """
        theta = np.asarray(theta, dtype=float)
        A = self.B + self.p.l1 * np.column_stack((np.cos(theta), np.sin(theta)))
        x = np.asarray(guess, dtype=float).copy()
        for _ in range(max_iter):
            C = self.platform_joints(x)
            u = C - A
            F = np.sum(u * u, axis=1) - self.p.l2**2
            if np.max(np.abs(F)) < tol:
                x[2] = float(wrap(x[2]))
                return x
            Rc = self.c @ rot2(x[2]).T
            J = 2.0 * np.column_stack((u[:, 0], u[:, 1], np.sum(u * (Rc @ E90.T), axis=1)))
            try:
                dx = np.linalg.solve(J, -F)
            except np.linalg.LinAlgError:
                return None
            x += dx
        return None

    # ------------------------------------------------------------- Jacobians
    def jacobians(self, pose: Sequence[float], ik: Optional[IKResult] = None):
        """Return (Jx, Jq) with  Jx @ xdot = Jq @ thetadot.

        Derived by differentiating the loop-closure constraint
        |C_i - A_i|^2 = l2^2  ->  u_i . (pdot + phidot E R c_i) = u_i . (thetadot_i E (A_i - B_i))
        """
        ik = ik or self.ik(pose)
        Rc = self.c @ rot2(pose[2]).T
        u = ik.platform_joints - ik.elbows
        Jx = np.column_stack((u[:, 0], u[:, 1], np.sum(u * (Rc @ E90.T), axis=1)))
        Jq = np.diag(np.sum(u * ((ik.elbows - self.B) @ E90.T), axis=1))
        return Jx, Jq

    def inverse_jacobian(self, pose, ik=None) -> np.ndarray:
        """thetadot = Jinv @ xdot  (always defined away from Type-I singularities)."""
        Jx, Jq = self.jacobians(pose, ik)
        return np.linalg.solve(Jq, Jx)

    def forward_jacobian(self, pose, ik=None) -> np.ndarray:
        """xdot = J @ thetadot (undefined at Type-II / parallel singularities)."""
        Jx, Jq = self.jacobians(pose, ik)
        return np.linalg.solve(Jx, Jq)

    def singularity_measures(self, pose, ik=None) -> dict:
        """Dimensionless measures used throughout the docs and scenarios.

        * ``kappa``  : condition number of the homogenised inverse Jacobian (1 = isotropic)
        * ``type1``  : min |Jq_ii| / (l1 l2)  -> 0 at a serial singularity (leg stretched/folded)
        * ``type2``  : |det Jx| / (l2^3 r)    -> 0 at a parallel singularity (platform loses stiffness)
        """
        ik = ik or self.ik(pose)
        Jx, Jq = self.jacobians(pose, ik)
        r = self.p.platform_radius
        t1 = float(np.min(np.abs(np.diag(Jq))) / (self.p.l1 * self.p.l2))
        t2 = float(abs(np.linalg.det(Jx)) / (self.p.l2**3 * r))
        try:
            Jinv = np.linalg.solve(Jq, Jx) @ np.diag([1.0, 1.0, 1.0 / r])
            kappa = float(np.linalg.cond(Jinv))
        except np.linalg.LinAlgError:
            kappa = float("inf")
        return {"kappa": kappa, "type1": t1, "type2": t2,
                "dexterity": 0.0 if not np.isfinite(kappa) else 1.0 / kappa}
