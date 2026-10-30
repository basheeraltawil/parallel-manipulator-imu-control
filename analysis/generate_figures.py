#!/usr/bin/env python3
"""Generate every figure used by the learning resources (docs/learning/*.md).

    python3 analysis/generate_figures.py            # all figures (~3 min)
    python3 analysis/generate_figures.py ik fk      # only some (names below)

Each function is short and commented on purpose: read it next to the lesson.
"""

from __future__ import annotations

import sys
from itertools import product
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "ros2_ws" / "src" / "pm3rrr_core"))
sys.path.insert(0, str(ROOT / "simulation"))

import matplotlib  # noqa: E402

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

from pm3rrr_core import Pm3rrrDynamics, Pm3rrrKinematics, rot2  # noqa: E402
from pm3rrr_core import trajectory as tr  # noqa: E402
from pm3rrr_core.imu import ComplementaryFilter, Mpu6050Model, accel_tilt  # noqa: E402
from pm3rrr_core.planning import assembly_sign  # noqa: E402
from pm3rrr_core.workspace import constant_orientation, largest_inscribed_circle  # noqa: E402
from pm3rrr_sim.plot2d import LEG_COLORS, draw_robot, setup_topview  # noqa: E402

OUT = ROOT / "docs" / "media" / "learning"
OUT.mkdir(parents=True, exist_ok=True)
kin = Pm3rrrKinematics()
plt.rcParams.update({"figure.dpi": 110, "axes.grid": True, "grid.alpha": 0.25, "font.size": 9})


def save(fig, name):
    fig.tight_layout()
    fig.savefig(OUT / name)
    plt.close(fig)
    print("  wrote", (OUT / name).relative_to(ROOT))


# --------------------------------------------------------------------- IK
def fig_ik():
    """One leg: the triangle B-A-C, the two elbow solutions, alpha, beta, theta."""
    pose = (0.004, 0.006, np.radians(8))
    fig, axs = plt.subplots(1, 2, figsize=(11, 5))
    ax = axs[0]
    setup_topview(ax, 105, "Whole robot: 3 legs, each B_i -> A_i -> C_i")
    draw_robot(ax, kin, pose)
    for i in range(3):
        ax.annotate(f"B{i+1}", kin.B[i] * 1e3 + [4, 4], fontsize=9, weight="bold")
    ax = axs[1]
    i = 0
    ik_m = kin.ik(pose, modes=(-1, -1, -1))
    ik_p = kin.ik(pose, modes=(1, 1, 1))
    B, C = kin.B[i] * 1e3, ik_m.platform_joints[i] * 1e3
    for ik, ls, lab in ((ik_m, "-", r"$\sigma=-1$:  $\theta=\beta-\alpha$  (used)"),
                        (ik_p, "--", r"$\sigma=+1$:  $\theta=\beta+\alpha$")):
        A = ik.elbows[i] * 1e3
        ax.plot(*np.c_[B, A, C], ls, color=LEG_COLORS[0], lw=3, label=lab)
        ax.plot(*A, "ko")
    ax.plot(*np.c_[B, C], ":", color="k", lw=1)
    ax.plot(*B, "s", color="#444", ms=10)
    ax.plot(*C, "o", color="#d62728", ms=8)
    d = C - B
    beta = np.arctan2(d[1], d[0])
    th = ik_m.theta[i]
    for ang0, ang1, r, txt, col in ((0, beta, 22, r"$\beta$", "#2ca02c"), (th, beta, 32, r"$\alpha$", "#ff7f0e"),
                                    (0, th, 12, r"$\theta$", "#1f77b4")):
        a = np.linspace(ang0, ang1, 40)
        ax.plot(B[0] + r * np.cos(a), B[1] + r * np.sin(a), color=col, lw=1.5)
        ax.text(B[0] + (r + 3) * np.cos(a[20]), B[1] + (r + 3) * np.sin(a[20]), txt, color=col, fontsize=13)
    ax.plot([B[0], B[0] + 40], [B[1], B[1]], "k-", lw=0.8)
    mid = (B + C) / 2
    ax.text(*(mid + [2, -6]), r"$d=|C-B|$", fontsize=10)
    ax.text(*(B + [-12, -6]), "B (motor)", fontsize=9)
    ax.text(*(C + [2, 3]), "C (platform joint)", fontsize=9)
    ax.set_aspect("equal")
    ax.legend(loc="upper right", fontsize=8)
    ax.set_title("Leg 1: law of cosines  " + r"$\cos\alpha=\frac{l_1^2+d^2-l_2^2}{2 l_1 d}$")
    pts = np.vstack((B, C, ik_p.elbows[i] * 1e3, ik_m.elbows[i] * 1e3))
    ax.set_xlim(pts[:, 0].min() - 25, pts[:, 0].max() + 25)
    ax.set_ylim(pts[:, 1].min() - 15, pts[:, 1].max() + 15)
    save(fig, "fig01_ik_geometry.png")


def fig_working_modes():
    """The 2^3 = 8 working modes (elbow branches) for the SAME platform pose."""
    pose = (0.0, 0.0, 0.0)
    fig, axs = plt.subplots(2, 4, figsize=(14, 7.2))
    for ax, modes in zip(axs.flat, product((-1, 1), repeat=3)):
        ik = kin.ik(pose, modes)
        k = kin.singularity_measures(pose, ik)["kappa"]
        setup_topview(ax, 100)
        draw_robot(ax, kin, pose, ik)
        tag = "  <- used" if modes == (-1, -1, -1) else ""
        ax.set_title(f"modes {''.join('+' if m > 0 else '-' for m in modes)}   kappa = {k:.2f}{tag}",
                     fontsize=9, color="#c62828" if tag else "k")
        ax.set_xlabel("")
        ax.set_ylabel("")
    fig.suptitle("Working modes: each leg's elbow can be on either side (8 IK solutions)", weight="bold")
    save(fig, "fig02_working_modes.png")


def fig_fk():
    """Forward kinematics: same motor angles, several platform poses (assembly modes)."""
    pose = (0.005, -0.004, np.radians(-10))
    theta = kin.ik(pose).theta
    sols = kin.fk_all(theta)
    fig, axs = plt.subplots(1, len(sols), figsize=(5 * len(sols), 5))
    axs = np.atleast_1d(axs)
    for ax, s in zip(axs, sols):
        setup_topview(ax, 100)
        A = kin.B + kin.p.l1 * np.c_[np.cos(theta), np.sin(theta)]
        C = kin.platform_joints(s)
        for i in range(3):
            ax.plot(*np.c_[kin.B[i], A[i]].T.T * 1e3, "-", color=LEG_COLORS[i], lw=3)
            ax.plot(*np.c_[A[i], C[i]].T.T * 1e3, "-", color=LEG_COLORS[i], lw=2)
        tri = np.vstack((C, C[:1])) * 1e3
        ax.fill(tri[:, 0], tri[:, 1], color="#d62728", alpha=0.35)
        sg = "+" if assembly_sign(kin, s) > 0 else "-"
        ax.set_title(f"x={s[0]*1e3:.1f} mm, y={s[1]*1e3:.1f} mm, phi={np.degrees(s[2]):.1f} deg\n"
                     f"sign(det Jx) = {sg}", fontsize=9)
    fig.suptitle(f"FK: motor angles {np.degrees(theta).round(1)} deg have {len(sols)} real assembly modes",
                 weight="bold")
    save(fig, "fig03_fk_assembly_modes.png")


def fig_workspace():
    fig, axs = plt.subplots(1, 3, figsize=(15, 5))
    for ax, ph in zip(axs, (-15, 0, 15)):
        w = constant_orientation(kin, np.radians(ph), 0.05, 121)
        im = ax.pcolormesh(w.xs * 1e3, w.ys * 1e3, np.log10(w.kappa), vmin=0, vmax=1.5, cmap="viridis_r",
                           shading="auto")
        r = largest_inscribed_circle(w, 5.0)
        ax.add_patch(plt.Circle((0, 0), r * 1e3, fill=False, ec="#d62728", lw=2, ls="--"))
        ax.set_aspect("equal")
        ax.set_title(f"phi = {ph:+d} deg: area {w.area*1e4:.1f} cm2, usable circle D = {2*r*1e3:.0f} mm", fontsize=9)
        ax.set_xlabel("x [mm]")
        ax.set_ylabel("y [mm]")
    fig.colorbar(im, ax=axs, label="log10(condition number)", shrink=0.8)
    fig.suptitle("Constant-orientation workspace (legs close AND servos within +-85 deg); "
                 "red = largest well-conditioned circle (kappa <= 5)", weight="bold", fontsize=10)
    fig.savefig(OUT / "fig04_workspace.png", bbox_inches="tight")
    plt.close(fig)
    print("  wrote docs/media/learning/fig04_workspace.png")


def fig_singularities():
    phis = (0, -20, -25, -30, -40)
    fig, axs = plt.subplots(2, len(phis), figsize=(18, 7.5))
    for j, ph in enumerate(phis):
        w = constant_orientation(kin, np.radians(ph), 0.045, 101, check_limits=False)
        ax = axs[0, j]
        im = ax.pcolormesh(w.xs * 1e3, w.ys * 1e3, np.log10(w.kappa), vmin=0, vmax=2, cmap="magma_r", shading="auto")
        ax.set_title(f"phi = {ph} deg", fontsize=10)
        ax.set_aspect("equal")
        S = np.full_like(w.kappa, np.nan)
        for a, y in enumerate(w.ys):
            for b, x in enumerate(w.xs):
                if w.reachable[a, b]:
                    S[a, b] = assembly_sign(kin, (x, y, np.radians(ph)))
        ax = axs[1, j]
        ax.pcolormesh(w.xs * 1e3, w.ys * 1e3, S, cmap="coolwarm", vmin=-1, vmax=1, shading="auto")
        ax.set_aspect("equal")
        ax.set_title("sign(det Jx): blue -, red +", fontsize=9)
    fig.colorbar(im, ax=axs[0].tolist(), label="log10 kappa", shrink=0.8)
    fig.suptitle("Type-II (parallel) singularities: dark curves = det(Jx)=0. Between -25 and -30 deg they sweep "
                 "through the workspace and the assembly mode flips", weight="bold", fontsize=10)
    fig.savefig(OUT / "fig05_singularities.png", bbox_inches="tight")
    plt.close(fig)
    print("  wrote docs/media/learning/fig05_singularities.png")


def fig_orientation_range():
    from pm3rrr_core.workspace import orientation_range
    xs, ys, lo, hi = orientation_range(kin, 0.035, 41, np.radians(60), 121, kappa_max=10)
    fig, axs = plt.subplots(1, 2, figsize=(11, 4.6))
    for ax, data, t in ((axs[0], np.degrees(lo), "min reachable phi [deg]"),
                        (axs[1], np.degrees(hi - lo), "orientation range [deg]")):
        im = ax.pcolormesh(xs * 1e3, ys * 1e3, data, cmap="viridis", shading="auto")
        fig.colorbar(im, ax=ax, label=t)
        ax.set_aspect("equal")
        ax.set_xlabel("x [mm]")
        ax.set_ylabel("y [mm]")
        ax.set_title(t)
    fig.suptitle("Orientation workspace (kappa <= 10, servo limits, same assembly mode as home)", weight="bold")
    save(fig, "fig06_orientation_workspace.png")


def fig_trajectory():
    tau = np.linspace(0, 1, 400)
    fig, axs = plt.subplots(1, 3, figsize=(13, 3.8))
    for name, col in (("cubic", "#1f77b4"), ("quintic", "#d62728"), ("trapezoidal", "#2ca02c")):
        s, ds, dds = tr.SCALINGS[name](tau)
        for ax, y in zip(axs, (s, ds, dds)):
            ax.plot(tau, y, color=col, lw=2, label=name)
    for ax, t in zip(axs, ("position s", "velocity ds/dtau", "acceleration d2s/dtau2")):
        ax.set_title(t)
        ax.set_xlabel("normalised time tau = t/T")
    axs[0].legend()
    fig.suptitle("Time-scaling laws: cubic (acc. jumps), quintic/min-jerk (smooth), trapezoidal (fastest for vmax)",
                 weight="bold", fontsize=10)
    save(fig, "fig07_trajectory_profiles.png")


def fig_task_vs_joint():
    a, b = np.array([-0.015, -0.010, 0.0]), np.array([0.014, 0.010, np.radians(8)])
    qa = kin.motor_to_joint(kin.ik(a).theta)
    qb = kin.motor_to_joint(kin.ik(b).theta)
    s = np.linspace(0, 1, 60)
    joint_path, guess = [], a.copy()
    for si in s:
        th = kin.joint_to_motor(qa + si * (qb - qa))
        guess = kin.fk(th, guess)
        joint_path.append(guess)
    joint_path = np.array(joint_path)
    fig, axs = plt.subplots(1, 2, figsize=(11, 4.4))
    ax = axs[0]
    ax.plot([a[0] * 1e3, b[0] * 1e3], [a[1] * 1e3, b[1] * 1e3], "-", lw=2.5, label="task-space interpolation (straight)")
    ax.plot(joint_path[:, 0] * 1e3, joint_path[:, 1] * 1e3, "--", lw=2.5, label="joint-space interpolation (curved!)")
    ax.set_aspect("equal")
    ax.legend(fontsize=8)
    ax.set_xlabel("x [mm]")
    ax.set_ylabel("y [mm]")
    ax.set_title("Same start/goal, different paths")
    ax = axs[1]
    traj = tr.sample(tr.line(a, b), 1.0, 0.005, "quintic")
    q, qd, qdd, ok = tr.to_joint_space(kin, traj)
    for i in range(3):
        ax.plot(traj.t, np.degrees(qd[:, i]), color=LEG_COLORS[i], lw=2, label=f"motor {i+1}")
    ax.set_xlabel("t [s]")
    ax.set_ylabel("joint velocity [deg/s]")
    ax.set_title("Straight line in 1 s (quintic): motor speeds")
    ax.legend(fontsize=8)
    save(fig, "fig08_task_vs_joint_space.png")


def fig_dynamics():
    fig, axs = plt.subplots(1, 3, figsize=(15, 4.3))
    dyn = Pm3rrrDynamics(kin)
    ax = axs[0]
    for T, ls in ((2.0, ":"), (1.0, "--"), (0.5, "-")):
        traj = tr.sample(tr.circle((0, 0), 0.015), T, 0.002, "quintic")
        tau = np.array([dyn.inverse_dynamics(x, xd, xdd) for x, xd, xdd in zip(traj.x, traj.xd, traj.xdd)])
        for i in range(3):
            ax.plot(traj.t / T, tau[:, i] * 1000, ls, color=LEG_COLORS[i], lw=1.6,
                    label=f"T={T}s" if i == 0 else None)
    ax.set_xlabel("t / T")
    ax.set_ylabel("motor torque [mN m]")
    ax.set_title("Inverse dynamics: circle D=30 mm (no load)")
    ax.legend(fontsize=8)
    ax = axs[1]
    speeds, peaks, peaks_load = [], [], []
    for T in np.linspace(0.25, 2.0, 15):
        traj = tr.sample(tr.circle((0, 0), 0.015), T, 0.004, "quintic")
        tau = [dyn.inverse_dynamics(x, xd, xdd) for x, xd, xdd in zip(traj.x, traj.xd, traj.xdd)]
        tau_l = [dyn.inverse_dynamics(x, xd, xdd, wrench=(-0.2 * xdd[0], -0.2 * xdd[1], 0))
                 for x, xd, xdd in zip(traj.x, traj.xd, traj.xdd)]
        speeds.append(np.abs(traj.xd[:, :2]).max() * 1e3)
        peaks.append(np.abs(tau).max() * 1000)
        peaks_load.append(np.abs(tau_l).max() * 1000)
    ax.semilogy(speeds, peaks, "o-", label="no payload")
    ax.semilogy(speeds, peaks_load, "s-", label="200 g payload")
    ax.axhline(1080, color="k", ls="--", lw=1)
    ax.text(speeds[0], 1150, "MG996R stall 1.08 N m", fontsize=8)
    ax.axhline(220, color="#888", ls="--", lw=1)
    ax.text(speeds[0], 235, "MG90S stall 0.22 N m", fontsize=8)
    ax.set_xlabel("peak platform speed [mm/s]")
    ax.set_ylabel("peak torque [mN m]")
    ax.set_title("Actuator sizing (inertial loads only)")
    ax.legend(fontsize=8)
    ax = axs[2]
    w = constant_orientation(kin, 0.0, 0.045, 61)
    F = np.full_like(w.kappa, np.nan)
    for a_, y in enumerate(w.ys):
        for b_, x in enumerate(w.xs):
            if w.reachable[a_, b_]:
                F[a_, b_] = dyn.max_static_wrench((x, y, 0.0), 1.08 * 0.5)
    im = ax.pcolormesh(w.xs * 1e3, w.ys * 1e3, F, cmap="YlGn", shading="auto", vmax=np.nanpercentile(F, 95))
    fig.colorbar(im, ax=ax, label="force [N]")
    ax.set_aspect("equal")
    ax.set_title("Guaranteed push force (any direction)\nwith 50 % of MG996R stall torque, phi = 0")
    ax.set_xlabel("x [mm]")
    save(fig, "fig09_dynamics.png")


def fig_imu():
    imu = Mpu6050Model(seed=5)
    dt = 0.01
    t = np.arange(0, 20, dt)
    roll = np.radians(25) * np.sin(2 * np.pi * 0.2 * (t - 2)) * (t > 2)
    lin = np.zeros((len(t), 3))
    lin[(t > 9) & (t < 11), 1] = 3.0 * np.sign(np.sin(2 * np.pi * 3 * t[(t > 9) & (t < 11)]))  # sideways shake
    rate = np.gradient(roll, dt)
    cf = ComplementaryFilter(0.5, dt)
    cf.calibrate([imu.measure(0, 0, np.zeros(3))[1] for _ in range(100)])
    acc_only, gyro_only, comp = [], [], []
    g_int = 0.0
    for k in range(len(t)):
        a, g = imu.measure(roll[k], 0.0, np.array([rate[k], 0, 0]), lin[k])
        acc_only.append(accel_tilt(a)[0])
        g_int += (g[0]) * dt                       # no bias removal -> drift
        gyro_only.append(g_int)
        comp.append(cf.update(a, g, dt)[0])
    fig, ax = plt.subplots(figsize=(11, 4))
    ax.plot(t, np.degrees(roll), "k", lw=2.5, label="true roll")
    ax.plot(t, np.degrees(acc_only), color="#ff7f0e", lw=0.8, alpha=0.8, label="accelerometer only (noisy, fooled by the shake)")
    ax.plot(t, np.degrees(gyro_only), color="#9467bd", lw=1.5, label="gyro integration only (drifts)")
    ax.plot(t, np.degrees(comp), color="#2ca02c", lw=2, label="complementary filter")
    ax.axvspan(9, 11, color="#ffcdd2", alpha=0.5)
    ax.text(9.1, 28, "linear shake", fontsize=8)
    ax.set_xlabel("t [s]")
    ax.set_ylabel("roll [deg]")
    ax.legend(fontsize=8, loc="lower left")
    ax.set_title("MPU6050 sensor fusion: why we need BOTH sensors")
    save(fig, "fig10_imu_fusion.png")


def fig_accuracy():
    """Platform error caused by a 0.5 deg joint error (servo dead band), worst case."""
    w = constant_orientation(kin, 0.0, 0.045, 61)
    P = np.full_like(w.kappa, np.nan)
    R = np.full_like(w.kappa, np.nan)
    dq = np.radians(0.5)
    corners = np.array(list(product((-1, 1), repeat=3))) * dq
    for a_, y in enumerate(w.ys):
        for b_, x in enumerate(w.xs):
            if w.reachable[a_, b_]:
                J = kin.forward_jacobian((x, y, 0.0))
                e = corners @ J.T
                P[a_, b_] = np.linalg.norm(e[:, :2], axis=1).max() * 1e3
                R[a_, b_] = np.degrees(np.abs(e[:, 2]).max())
    fig, axs = plt.subplots(1, 2, figsize=(11, 4.5))
    for ax, data, lab in ((axs[0], P, "position error [mm]"), (axs[1], R, "orientation error [deg]")):
        im = ax.pcolormesh(w.xs * 1e3, w.ys * 1e3, data, cmap="inferno_r", shading="auto",
                           vmax=np.nanpercentile(data, 95))
        fig.colorbar(im, ax=ax, label=lab)
        ax.set_aspect("equal")
        ax.set_xlabel("x [mm]")
        ax.set_title(f"worst-case {lab}\n(each servo +-0.5 deg, phi = 0)")
    save(fig, "fig11_accuracy_map.png")


def gif_ik():
    """2-D animation: the IK triangle construction while the platform moves."""
    from pm3rrr_sim.media import fig_to_array, save_gif
    frames = []
    traj = tr.sample(tr.lemniscate((0, 0), 0.018, phi_amp=np.radians(12)), 1.0, 1 / 48, "quintic")
    fig, ax = plt.subplots(figsize=(5.2, 5.2), dpi=90)
    for k, x in enumerate(traj.x):
        ax.cla()
        setup_topview(ax, 100, "Inverse kinematics: pose (x, y, phi) -> motor angles")
        ik = kin.ik(x)
        draw_robot(ax, kin, x, ik)
        for i in range(3):
            ax.plot(*np.c_[kin.B[i], ik.platform_joints[i]] * 1e3, ":", color=LEG_COLORS[i], lw=1)
        ax.plot(traj.x[: k + 1, 0] * 1e3, traj.x[: k + 1, 1] * 1e3, color="#f2a900", lw=2)
        q = np.degrees(kin.motor_to_joint(ik.theta)) + 90
        ax.text(-98, -98, "servo cmd [deg]: " + "  ".join(f"{v:5.1f}" for v in q), fontsize=8, family="monospace")
        frames.append(fig_to_array(fig))
    plt.close(fig)
    save_gif(frames, OUT / "ik_animation.gif", fps=16, max_colors=64)


def fig_render():
    """Static render of the URDF in MuJoCo for the docs."""
    from pm3rrr_sim.mujoco_model import MujocoPm3rrr
    from PIL import Image
    sim = MujocoPm3rrr(width=900, height=640)
    sim.set_pose((0.0, 0.0, 0.0))
    Image.fromarray(sim.render(azimuth=130, elevation=-30, distance=0.38, lookat=(0, 0, 0.06))).save(
        OUT / "robot_render.png")
    Image.fromarray(sim.render(azimuth=90, elevation=-89, distance=0.36, lookat=(0, 0, 0.06))).save(
        OUT / "robot_render_top.png")
    print("  wrote docs/media/learning/robot_render*.png")


ALL = {"ik": fig_ik, "modes": fig_working_modes, "fk": fig_fk, "workspace": fig_workspace,
       "singularities": fig_singularities, "orientation": fig_orientation_range, "trajectory": fig_trajectory,
       "taskjoint": fig_task_vs_joint, "dynamics": fig_dynamics, "imu": fig_imu, "accuracy": fig_accuracy,
       "ikgif": gif_ik, "render": fig_render}

if __name__ == "__main__":
    for name in (sys.argv[1:] or ALL):
        print(name)
        ALL[name]()
