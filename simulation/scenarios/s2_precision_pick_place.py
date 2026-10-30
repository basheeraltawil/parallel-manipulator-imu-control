#!/usr/bin/env python3
"""Scenario 2 - Precision positioning: visiting the wells of a micro-plate.

Real problem: lab automation / inspection. A small XY(θ) stage must stop
precisely over the wells of a standard micro-plate (9 mm pitch) so a
pipette, camera or probe can work. Throughput (moves/second) and accuracy
at each stop are the KPIs.

What the scenario teaches:
  * rest-to-rest quintic (minimum-jerk) moves with velocity/acceleration limits
  * accuracy is limited by the ACTUATOR, not by the maths:
      Servo.write(int deg)        -> 1.0 deg command steps  (original firmware)
      Servo.writeMicroseconds(us) -> 0.18 deg steps         (new firmware)
      ...but the hobby servo's ~0.5 deg DEAD BAND remains -> a smart servo
      with an encoder (0.088 deg, tiny dead band) is the industrial fix.
    The Jacobian turns joint errors into platform error: dx = J dq.

Run:  python3 simulation/scenarios/s2_precision_pick_place.py [--no-media]
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
from pm3rrr_sim.mujoco_model import MujocoPm3rrr  # noqa: E402
from pm3rrr_sim.plot2d import draw_robot, setup_topview  # noqa: E402
from pm3rrr_sim.runner import run  # noqa: E402

from pm3rrr_core import trajectory as tr  # noqa: E402

PITCH = 0.009                      # SBS/ANSI micro-plate well pitch: 9 mm
WELLS = [(-PITCH, PITCH / 2), (0, PITCH / 2), (PITCH, PITCH / 2),
         (PITCH, -PITCH / 2), (0, -PITCH / 2), (-PITCH, -PITCH / 2)]
DWELL = 0.5


def build_trajectory():
    pts = [(0.0, 0.0, 0.0)] + [(x, y, 0.0) for x, y in WELLS] + [(0.0, 0.0, 0.0)]
    traj = tr.point_to_point(pts, vmax=0.06, amax=0.6, dt=0.01, profile="quintic", dwell=DWELL)
    return pts, traj


def stop_errors(log, pts, traj):
    """Platform error at the END of every dwell (when the pipette would act)."""
    L = log.arrays()
    errs = []
    t_dw = []
    # dwell ends = times when the trajectory has been at a well for DWELL seconds
    at = np.linalg.norm(np.diff(traj.x[:, :2], axis=0), axis=1) < 1e-12
    idx = np.flatnonzero(np.diff(at.astype(int)) == -1)    # end of each still segment
    for k in idx:
        t = traj.t[k]
        j = int(np.argmin(np.abs(L["t"] - t)))
        errs.append(np.linalg.norm(L["actual"][j, :2] - L["target"][j, :2]))
        t_dw.append(t)
    return np.array(errs[:len(WELLS)]), np.array(t_dw[:len(WELLS)])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--no-media", action="store_true")
    a = ap.parse_args()
    pts, traj = build_trajectory()
    T = traj.t[-1]

    def target(t):
        return traj.x[min(int(round(t / 0.01)), len(traj) - 1)]

    results = {}
    for mode in ("deg", "smart", "us"):
        sim = MujocoPm3rrr(width=480, height=400)
        sim.set_pose((0, 0, 0))
        if mode != "us" or a.no_media:
            log, _ = run(sim, target, T, servo_mode=mode)
            results[mode] = (log, *stop_errors(log, pts, traj))
            continue

        fig = plt.figure(figsize=(4.8, 4.0), dpi=100)
        ax1 = fig.add_axes([0.13, 0.42, 0.83, 0.50])
        ax2 = fig.add_axes([0.13, 0.09, 0.83, 0.22])
        err_deg = results["deg"][1]
        err_smart = results["smart"][1]
        z = 0.0808 + 0.016 + 0.0075

        def on_frame(k, t, log):
            act = np.asarray(log.actual)
            marks = [((x, y, z - 0.0005), (0.1, 0.8, 0.3, 1), 0.0016) for x, y in WELLS]
            img3d = sim.render(azimuth=120, elevation=-55, distance=0.36, lookat=(0, 0, 0.08),
                               trail=act[-60:, :2], markers=marks)
            ax1.cla()
            setup_topview(ax1, lim=18)
            for i, (x, y) in enumerate(WELLS):
                ax1.add_patch(plt.Circle((x * 1e3, y * 1e3), 3.2, fc="#e8f5e9", ec="#2ca02c"))
                ax1.text(x * 1e3, y * 1e3 + 4.2, f"W{i+1}", ha="center", fontsize=7, color="#2ca02c")
            ax1.plot(traj.x[:, 0] * 1e3, traj.x[:, 1] * 1e3, ":", color="#999", lw=1)
            ax1.plot(act[:, 0] * 1e3, act[:, 1] * 1e3, "-", color="#d62728", lw=1.5)
            ax1.plot(act[-1, 0] * 1e3, act[-1, 1] * 1e3, "o", color="#d62728", ms=6)
            ax1.set_title("S2  Micro-plate positioning (9 mm pitch)", fontsize=10, weight="bold")
            ax1.tick_params(labelsize=7)
            ax1.text(0.98, 0.03, f"t = {t:4.1f} s", transform=ax1.transAxes, ha="right", fontsize=8)
            ax2.cla()
            e_now, _ = stop_errors(log, pts, traj) if len(log.t) > 5 else (np.array([]), None)
            xs = np.arange(len(WELLS))
            ax2.bar(xs - 0.27, err_deg * 1e3, 0.27, color="#999", label="hobby, write(deg)")
            ax2.bar(xs[:len(e_now)], e_now * 1e3, 0.27, color="#d62728", label="hobby, writeMicroseconds")
            ax2.bar(xs + 0.27, err_smart * 1e3, 0.27, color="#1f77b4", label="smart servo")
            ax2.set_xticks(xs, [f"W{i+1}" for i in xs], fontsize=7)
            ax2.set_ylabel("stop error [mm]", fontsize=7)
            ax2.set_ylim(0, max(0.5, 1.2 * err_deg.max() * 1e3))
            ax2.tick_params(labelsize=7)
            ax2.legend(fontsize=5.5, loc="upper right", ncol=3)
            return hstack(img3d, fig_to_array(fig))

        log, frames = run(sim, target, T, servo_mode=mode, frame_every=3, on_frame=on_frame)
        results[mode] = (log, *stop_errors(log, pts, traj))

    print("Scenario 2 - micro-plate positioning")
    print(f"  cycle time for 6 wells         : {T:.2f} s  ({len(WELLS) / T * 60:.0f} wells/min incl. {DWELL}s dwell)")
    for mode, name in (("deg", "hobby  Servo.write(int deg) "), ("us", "hobby  writeMicroseconds(us)"),
                       ("smart", "smart servo (encoder, 0.088)")):
        log, e, _ = results[mode]
        L = log.arrays()
        print(f"  {name}: stop error mean {1e3*e.mean():.3f} mm, max {1e3*e.max():.3f} mm, "
              f"peak torque {100*np.abs(L['tau']).max():.1f} N cm")
    if not a.no_media:
        save_gif(frames, MEDIA / "scenarios" / "s2_precision_pick_place.gif", fps=15)
        save_mp4(frames, MEDIA / "videos" / "s2_precision_pick_place.mp4", fps=15)


if __name__ == "__main__":
    main()
