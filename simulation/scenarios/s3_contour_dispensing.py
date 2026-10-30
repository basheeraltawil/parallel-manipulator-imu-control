#!/usr/bin/env python3
"""Scenario 3 - Contour following: dispensing a glue bead around a part.

Real problem: dispensing, laser engraving, cutting or inspection along a
contour. The nozzle must follow a rounded-rectangle outline at a CONSTANT
speed (bead width depends on speed!). KPIs: contour error and speed
uniformity. We sweep the feed rate to find the fastest speed that keeps
the contour error below a tolerance (here 0.25 mm).

Teaches: arc-length parameterisation, trapezoidal feed-rate profile,
tracking lag of position-controlled servos, choosing a feed rate from data.

Run:  python3 simulation/scenarios/s3_contour_dispensing.py [--no-media]
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
from pm3rrr_sim.plot2d import setup_topview  # noqa: E402
from pm3rrr_sim.runner import run  # noqa: E402

from pm3rrr_core import trajectory as tr  # noqa: E402

TOL = 0.5e-3          # +-0.5 mm: typical for a 1 mm glue bead


def rounded_rectangle(w=0.028, h=0.018, r=0.004, n=800):
    """Closed outline of the part, as dense points (start at the middle of the bottom edge)."""
    pts = []
    cx, cy = w / 2 - r, h / 2 - r
    for (ox, oy, a0) in ((cx, -cy, -90), (cx, cy, 0), (-cx, cy, 90), (-cx, -cy, 180)):
        a = np.radians(np.linspace(a0, a0 + 90, 40))
        pts.append(np.column_stack((ox + r * np.cos(a), oy + r * np.sin(a))))
    P = np.vstack([np.array([[0.0, -h / 2]])] + pts + [np.array([[0.0, -h / 2]])])
    s = np.concatenate(([0], np.cumsum(np.linalg.norm(np.diff(P, axis=0), axis=1))))
    su = np.linspace(0, s[-1], n)
    return np.column_stack((np.interp(su, s, P[:, 0]), np.interp(su, s, P[:, 1]))), s[-1]


def contour_trajectory(speed, dt=0.01, ta=0.08):
    P, length = rounded_rectangle()
    T = length / (speed * (1 - ta))
    t = np.arange(0, T + dt, dt)
    s, _, _ = tr.trapezoidal(t / T, ta)
    xy = np.column_stack((np.interp(s * length, np.linspace(0, length, len(P)), P[:, 0]),
                          np.interp(s * length, np.linspace(0, length, len(P)), P[:, 1])))
    return t, np.column_stack((xy, np.zeros(len(t)))), P, length


def contour_error(actual_xy, P):
    """Distance from each actual point to the nearest point of the ideal contour."""
    d = np.linalg.norm(actual_xy[:, None, :] - P[None, :, :], axis=2)
    return d.min(axis=1)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--no-media", action="store_true")
    a = ap.parse_args()
    speeds = [0.010, 0.020, 0.040, 0.060, 0.080]
    stats = []
    media_speed = 0.040
    frames = []
    for v in speeds:
        t, X, P, length = contour_trajectory(v)
        sim = MujocoPm3rrr(width=480, height=400)
        sim.set_pose(X[0])

        def target(tt, X=X, t=t):
            return X[min(int(round(tt / 0.01)), len(X) - 1)]

        on_frame = None
        if v == media_speed and not a.no_media:
            fig = plt.figure(figsize=(4.8, 4.0), dpi=100)
            ax1 = fig.add_axes([0.13, 0.42, 0.83, 0.50])
            ax2 = fig.add_axes([0.13, 0.09, 0.83, 0.22])

            def on_frame(k, tt, log, P=P):
                act = np.asarray(log.actual)
                img3d = sim.render(azimuth=100, elevation=-50, distance=0.34, lookat=(0, 0, 0.085),
                                   trail=act[:, :2], trail_rgba=(1.0, 0.75, 0.05, 1.0))
                ax1.cla()
                setup_topview(ax1, lim=19)
                ax1.add_patch(plt.Rectangle((-12, -7), 24, 14, fc="#ddd", ec="#888"))
                ax1.plot(P[:, 0] * 1e3, P[:, 1] * 1e3, "--", color="#555", lw=1, label="programmed contour")
                ax1.plot(act[:, 0] * 1e3, act[:, 1] * 1e3, "-", color="#f2a900", lw=2.5, label="glue bead (physics)")
                ax1.legend(fontsize=6, loc="upper right")
                ax1.set_title(f"S3  Contour dispensing @ {v*1e3:.0f} mm/s", fontsize=10, weight="bold")
                ax1.tick_params(labelsize=7)
                ax2.cla()
                e = contour_error(act[:, :2], P) * 1e3
                ax2.plot(np.asarray(log.t), e, color="#d62728", lw=1.2)
                ax2.axhline(TOL * 1e3, color="k", ls=":", lw=1)
                ax2.text(0.2, TOL * 1e3 * 1.05, "tolerance", fontsize=6)
                ax2.set_xlim(0, t[-1])
                ax2.set_ylim(0, 1.0)
                ax2.set_ylabel("contour err [mm]", fontsize=7)
                ax2.set_xlabel("t [s]", fontsize=7)
                ax2.tick_params(labelsize=7)
                return hstack(img3d, fig_to_array(fig))

        log, fr = run(sim, target, t[-1], servo_mode="us", frame_every=3, on_frame=on_frame)
        if fr:
            frames = fr
        L = log.arrays()
        e = contour_error(L["actual"][:, :2], P)
        sp = np.linalg.norm(np.diff(L["actual"][:, :2], axis=0), axis=1) / 0.02
        cruise = (L["t"][1:] > 0.2 * t[-1]) & (L["t"][1:] < 0.8 * t[-1])
        stats.append((v, t[-1], np.sqrt(np.mean(e**2)), e.max(), sp[cruise].std() / v))

    print("Scenario 3 - contour dispensing (rounded rectangle 28 x 18 mm, perimeter "
          f"{rounded_rectangle()[1]*1e3:.1f} mm)")
    print("  speed[mm/s]  cycle[s]  RMS err[mm]  max err[mm]  speed ripple   within tol?")
    best = None
    for v, T, rms, mx, rip in stats:
        ok = mx <= TOL
        best = v if ok else best
        print(f"   {v*1e3:6.0f}     {T:6.2f}     {rms*1e3:7.3f}      {mx*1e3:7.3f}      {100*rip:5.1f} %       {'yes' if ok else 'NO'}")
    print(f"  -> fastest feed rate within {TOL*1e3:.2f} mm tolerance: {best*1e3:.0f} mm/s" if best else "  -> none")

    fig, ax = plt.subplots(figsize=(5, 3.2), dpi=110)
    ax.plot([s[0] * 1e3 for s in stats], [s[3] * 1e3 for s in stats], "o-", label="max contour error")
    ax.plot([s[0] * 1e3 for s in stats], [s[2] * 1e3 for s in stats], "s-", label="RMS contour error")
    ax.axhline(TOL * 1e3, color="k", ls=":", label="tolerance")
    ax.set_xlabel("feed rate [mm/s]")
    ax.set_ylabel("error [mm]")
    ax.set_title("S3: contour error vs. feed rate (physics model)")
    ax.grid(alpha=0.3)
    ax.legend()
    fig.tight_layout()
    out = MEDIA / "scenarios" / "s3_feedrate_vs_error.png"
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out)
    if frames:
        save_gif(frames, MEDIA / "scenarios" / "s3_contour_dispensing.gif", fps=15)
        save_mp4(frames, MEDIA / "videos" / "s3_contour_dispensing.mp4", fps=15)


if __name__ == "__main__":
    main()
