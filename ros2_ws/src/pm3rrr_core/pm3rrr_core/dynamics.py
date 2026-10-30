"""Rigid-body dynamics of the 3-RRR robot (see docs/learning/08_dynamics.md).

Method: *principle of virtual work* with task-space coordinates x = (px, py, phi).

Each of the 7 moving bodies (3 proximal links, 3 distal links, 1 platform)
plus the 3 servo rotors has a body Jacobian J_b(x) mapping xdot to the body's
(vx_cm, vy_cm, omega). Then

    Jinv^T tau = sum_b J_b^T ( M_b a_b - f_b ) - F_ext ,   a_b = J_b xdd + Jdot_b xd

which gives the actuator torques tau for a desired motion. Planar rigid
bodies described by their CoM and absolute angle have no Coriolis terms in
M_b a_b, so this is exact. The robot normally works in a horizontal plane:
gravity is then perpendicular to the motion and loads only the bearings
(``gravity=(0, 0)``). For a wall-mounted robot pass ``gravity=(0, -9.81)``.
"""

from __future__ import annotations

from typing import Sequence

import numpy as np

from .kinematics import E90, Pm3rrrKinematics, rot2


class Pm3rrrDynamics:
    def __init__(self, kin: Pm3rrrKinematics, gravity: Sequence[float] = (0.0, 0.0),
                 viscous_friction: float = 0.0):
        self.kin = kin
        self.p = kin.p
        self.g = np.asarray(gravity, float)
        self.b = viscous_friction
        p = self.p
        bodies = []
        for _ in range(3):
            bodies.append(np.diag([p.proximal.mass, p.proximal.mass, p.proximal.izz]))
        for _ in range(3):
            bodies.append(np.diag([p.distal.mass, p.distal.mass, p.distal.izz]))
        bodies.append(np.diag([p.platform_mass, p.platform_mass, p.platform_izz]))
        for _ in range(3):
            bodies.append(np.diag([0.0, 0.0, p.rotor_inertia]))
        self.M_bodies = bodies

    # ------------------------------------------------------------------ helpers
    def body_jacobians(self, x: Sequence[float]):
        """List of 10 (3x3) body Jacobians [vx_cm, vy_cm, omega] = J_b xdot."""
        kin, p = self.kin, self.p
        ik = kin.ik(x)
        Jinv = kin.inverse_jacobian(x, ik)            # thetadot = Jinv xdot
        A, C, B = ik.elbows, ik.platform_joints, kin.B
        Rc = kin.c @ rot2(x[2]).T
        Js = []
        # proximal links: CoM on segment B->A
        for i in range(3):
            e = (A[i] - B[i]) / p.l1
            vcm = np.outer(p.proximal.com * (E90 @ e), Jinv[i])
            Js.append(np.vstack((vcm, Jinv[i])))
        # distal links: CoM on segment A->C
        for i in range(3):
            Adot = np.outer(E90 @ (A[i] - B[i]), Jinv[i])                # 2x3
            Cdot = np.column_stack((np.eye(2), E90 @ Rc[i]))             # 2x3
            e = (C[i] - A[i]) / p.l2
            w = (E90 @ e) @ (Cdot - Adot) / p.l2                         # 1x3
            vcm = Adot + p.distal.com * np.outer(E90 @ e, w)
            Js.append(np.vstack((vcm, w)))
        Js.append(np.eye(3))                                             # platform (CoM at centre)
        for i in range(3):                                               # rotors
            Js.append(np.vstack((np.zeros((2, 3)), Jinv[i])))
        return Js, Jinv

    def mass_matrix(self, x) -> np.ndarray:
        """Task-space inertia matrix M_x(x)."""
        Js, _ = self.body_jacobians(x)
        return sum(J.T @ M @ J for J, M in zip(Js, self.M_bodies))

    def kinetic_energy(self, x, xd) -> float:
        xd = np.asarray(xd, float)
        return 0.5 * float(xd @ self.mass_matrix(x) @ xd)

    def potential_energy(self, x) -> float:
        if not np.any(self.g):
            return 0.0
        kin, p = self.kin, self.p
        ik = kin.ik(x)
        U = 0.0
        for i in range(3):
            e1 = (ik.elbows[i] - kin.B[i]) / p.l1
            U -= p.proximal.mass * self.g @ (kin.B[i] + p.proximal.com * e1)
            e2 = (ik.platform_joints[i] - ik.elbows[i]) / p.l2
            U -= p.distal.mass * self.g @ (ik.elbows[i] + p.distal.com * e2)
        U -= p.platform_mass * self.g @ np.asarray(x[:2])
        return float(U)

    # --------------------------------------------------------------- dynamics
    def inverse_dynamics(self, x, xd, xdd, wrench: Sequence[float] = (0.0, 0.0, 0.0)) -> np.ndarray:
        """Motor torques [N m] (world-angle convention) for the motion (x, xd, xdd).

        ``wrench`` = external force (Fx, Fy) and moment Mz acting ON the
        platform (e.g. a tool contact force or a payload's inertial load).
        """
        x, xd, xdd = (np.asarray(v, float) for v in (x, xd, xdd))
        Js, Jinv = self.body_jacobians(x)
        h = 1e-6
        if np.linalg.norm(xd) > 0:
            Jp, _ = self.body_jacobians(x + h * xd)
            Jm, _ = self.body_jacobians(x - h * xd)
        F = -np.asarray(wrench, float)
        for k, (J, M) in enumerate(zip(Js, self.M_bodies)):
            a = J @ xdd
            if np.linalg.norm(xd) > 0:
                a = a + (Jp[k] - Jm[k]) / (2 * h) @ xd
            fg = np.zeros(3)
            fg[:2] = M[0, 0] * self.g
            F += J.T @ (M @ a - fg)
        tau = np.linalg.solve(Jinv.T, F)
        if self.b:
            tau = tau + self.b * (Jinv @ xd)
        return tau

    def static_torques(self, x, wrench) -> np.ndarray:
        """Torques that hold a static external wrench: tau = Jinv^-T (-w)  (stiffness/force check)."""
        _, Jinv = self.body_jacobians(x)
        return np.linalg.solve(Jinv.T, -np.asarray(wrench, float))

    def max_static_wrench(self, x, tau_max: float):
        """Largest force [N] the platform can resist in the worst direction at pose x."""
        _, Jinv = self.body_jacobians(x)
        J = np.linalg.inv(Jinv.T)[:, :2]                   # tau = J f  (force part only)
        smax = np.linalg.svd(J, compute_uv=False).max()
        return tau_max / smax
