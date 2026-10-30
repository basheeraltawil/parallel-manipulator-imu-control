#!/usr/bin/env python3
"""Overview animation for the README: the robot draws a figure-eight while the camera orbits.

Run:  python3 simulation/scenarios/s0_overview.py
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from pm3rrr_sim.media import MEDIA, save_gif, save_mp4  # noqa: E402
from pm3rrr_sim.mujoco_model import MujocoPm3rrr  # noqa: E402
from pm3rrr_sim.runner import run  # noqa: E402

from pm3rrr_core import trajectory as tr  # noqa: E402


def main():
    T = 6.0
    traj = tr.sample(tr.lemniscate((0.0, 0.0), 0.016, phi_amp=np.radians(10)), T, 0.01, "quintic")
    sim = MujocoPm3rrr(width=560, height=400)
    sim.set_pose(traj.x[0])

    def on_frame(k, t, log):
        act = np.asarray(log.actual)
        return sim.render(azimuth=120 + 60 * t, elevation=-32, distance=0.40, lookat=(0, 0, 0.065),
                          trail=act[:, :2])

    log, frames = run(sim, lambda t: traj.x[min(int(round(t / 0.01)), len(traj) - 1)], T,
                      servo_mode="us", frame_every=4, on_frame=on_frame)
    save_gif(frames, MEDIA / "overview.gif", fps=12, max_colors=160, scale=0.9)
    save_mp4(frames, MEDIA / "videos" / "overview.mp4", fps=12)


if __name__ == "__main__":
    main()
