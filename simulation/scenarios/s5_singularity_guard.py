#!/usr/bin/env python3
"""Scenario 5 - Singularity protection: "rotate the part by 40 degrees".

Real problem: an operator (or a CAM program) asks the platform to rotate a
part by -40 deg. Both end poses look perfectly fine on their own, but the
straight path crosses a Type-II (parallel) singularity at about -25 deg,
and the goal lies in ANOTHER ASSEMBLY MODE of the mechanism.

  UNGUARDED  : the servos happily reach the motor angles of the goal, but the
               platform falls back to the original assembly mode and ends far
               from the goal - while the controller believes it is there.
  GUARDED    : pm3rrr_core.planning.guard_trajectory checks every sample
               (condition number, IK, sign of det Jx) and stops the motion at
               the last safe pose, raising a clear fault message.

A small 0.3 N tool force acts on the platform (any real process has one).

Run:  python3 simulation/scenarios/s5_singularity_guard.py [--no-media]
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import matplotlib  # noqa: E402

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from pm3rrr_sim.media import MEDIA, fig_to_array, hstack, save_gif, save_mp4  # noqa: E402
from pm3rrr_sim.mujoco_model import MujocoPm3rrr, mujoco  # noqa: E402
from pm3rrr_sim.runner import run  # noqa: E402

from pm3rrr_core import trajectory as tr  # noqa: E402
from pm3rrr_core.planning import assembly_sign, guard_trajectory  # noqa: E402

GOAL = np.array([0.0, 0.0, np.radians(-40.0)])
KAPPA_MAX = 10.0


def make_sim(w=320, h=400):
    sim = MujocoPm3rrr(width=w, height=h)
    sim.set_pose((0, 0, 0))
    bid = mujoco.mj_name2id(sim.model, mujoco.mjtObj.mjOBJ_BODY, "platform_link")
    sim.data.xfrc_applied[bid, :3] = [0.2, 0.2, 0.0]          # 0.28 N process force
    return sim


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--no-media", action="store_true")
    a = ap.parse_args()
    traj = tr.sample(tr.line((0, 0, 0), GOAL), 4.0, 0.01, "quintic")
    kin = make_sim(8, 8).kin
    k_stop, reason = guard_trajectory(kin, traj.x, KAPPA_MAX)
    guarded = traj.x.copy()
    guarded[k_stop + 1:] = guarded[k_stop]
    kappa_path = np.array([kin.singularity_measures(x)["kappa"] for x in traj.x])
    sign_path = np.array([assembly_sign(kin, x) for x in traj.x])
    T = 5.0

    sims = {"unguarded": make_sim(), "guarded": make_sim()}
    targets = {"unguarded": traj.x, "guarded": guarded}
    logs = {}
    fig = plt.figure(figsize=(3.2, 4.0), dpi=100)
    ax1 = fig.add_axes([0.2, 0.58, 0.75, 0.34])
    ax2 = fig.add_axes([0.2, 0.10, 0.75, 0.34])

    # run the guarded robot first (no media), then the unguarded one rendering both
    logs["guarded"], _ = run(sims["guarded"], lambda t: guarded[min(int(round(t / 0.01)), len(guarded) - 1)],
                             T, servo_mode="us")
    Lg = logs["guarded"].arrays()
    sims["guarded"] = make_sim()

    def on_frame(k, t, log):
        # replay the (already simulated) guarded robot at the same instant, side by side
        sg = sims["guarded"]
        sg.set_pose(Lg["actual"][min(k, len(Lg["actual"]) - 1)])
        imgs = []
        for name, s in (("UNGUARDED", sims["unguarded"]), ("GUARDED", sg)):
            img = s.render(azimuth=90, elevation=-70, distance=0.30, lookat=(0, 0, 0.08))
            imgs.append(_label(img, name, "#c62828" if name == "UNGUARDED" else "#2e7d32"))
        La = np.asarray(log.actual)
        tt = np.asarray(log.t)
        ax1.cla()
        ax1.plot(traj.t, np.degrees(traj.x[:, 2]), ":", color="k", lw=1, label="requested")
        ax1.plot(tt, np.degrees(La[:, 2]), color="#c62828", lw=1.8, label="unguarded (actual)")
        ax1.plot(Lg["t"][:len(tt)], np.degrees(Lg["actual"][:len(tt), 2]), color="#2e7d32", lw=1.8,
                 label="guarded (actual)")
        ax1.set_xlim(0, T)
        ax1.set_ylim(-45, 5)
        ax1.set_ylabel("platform phi [deg]", fontsize=7)
        ax1.legend(fontsize=5.5, loc="lower left")
        ax1.tick_params(labelsize=6)
        ax1.set_title("S5  Singularity guard", fontsize=9, weight="bold")
        ax2.cla()
        ax2.semilogy(np.degrees(traj.x[:, 2]), kappa_path, color="#555")
        ax2.axhline(KAPPA_MAX, color="#2e7d32", ls="--", lw=1)
        ax2.fill_between(np.degrees(traj.x[:, 2]), 1, 1e3, where=sign_path < 0, color="#ffcdd2", alpha=0.7)
        ax2.text(-38, 300, "other\nassembly\nmode", fontsize=6, color="#c62828")
        ax2.set_xlim(2, -42)
        ax2.set_ylim(1, 1e3)
        ax2.set_xlabel("requested phi [deg]", fontsize=7)
        ax2.set_ylabel("condition number", fontsize=7)
        ax2.tick_params(labelsize=6)
        return hstack(imgs[0], imgs[1], fig_to_array(fig))

    logs["unguarded"], frames = run(sims["unguarded"],
                                    lambda t: traj.x[min(int(round(t / 0.01)), len(traj) - 1)], T,
                                    servo_mode="us", frame_every=3,
                                    on_frame=None if a.no_media else on_frame)
    Lu = logs["unguarded"].arrays()
    q_goal = kin.motor_to_joint(kin.ik(GOAL).theta)
    others = kin.fk_all(kin.ik(GOAL).theta)
    print("Scenario 5 - singularity guard (rotate 0 -> -40 deg)")
    print(f"  worst condition number on the straight path : {kappa_path.max():.0f} at phi = "
          f"{np.degrees(traj.x[np.argmax(kappa_path), 2]):.1f} deg")
    print(f"  guard stopped at phi = {np.degrees(guarded[-1, 2]):.1f} deg  -> fault: {reason}")
    print(f"  UNGUARDED: motors reached goal angles (err {np.degrees(np.abs(Lu['q_cmd'][-1] - q_goal)).max():.2f} deg)"
          f" but platform ended at phi = {np.degrees(Lu['actual'][-1, 2]):.1f} deg "
          f"(error {np.degrees(abs(Lu['actual'][-1, 2] - GOAL[2])):.1f} deg)")
    print("  forward kinematics of those motor angles has", len(others), "solutions:",
          ", ".join(f"phi={np.degrees(o[2]):.1f}" for o in others))
    print(f"  GUARDED  : platform at phi = {np.degrees(Lg['actual'][-1, 2]):.1f} deg "
          f"(tracking error {np.degrees(abs(Lg['actual'][-1, 2] - guarded[-1, 2])):.2f} deg), safe stop")
    if frames:
        save_gif(frames, MEDIA / "scenarios" / "s5_singularity_guard.gif", fps=15)
        save_mp4(frames, MEDIA / "videos" / "s5_singularity_guard.mp4", fps=15)


def _label(img, text, color):
    f = plt.figure(figsize=(img.shape[1] / 100, img.shape[0] / 100), dpi=100)
    ax = f.add_axes([0, 0, 1, 1])
    ax.imshow(img)
    ax.axis("off")
    ax.text(0.5, 0.95, text, transform=ax.transAxes, ha="center", va="top", fontsize=11, weight="bold",
            color="white", bbox=dict(fc=color, ec="none", pad=3))
    out = fig_to_array(f)
    plt.close(f)
    return out


if __name__ == "__main__":
    main()
