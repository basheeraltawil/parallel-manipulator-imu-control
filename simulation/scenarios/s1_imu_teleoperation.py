#!/usr/bin/env python3
"""Scenario 1 - Gesture (IMU) teleoperation of the platform.

Real problem: an operator steers a small positioning stage (micro-manipulation,
camera/microscope positioning, rehabilitation or teaching aid) by tilting a
hand-held MPU6050 - the original idea of this project.

We compare two pipelines on the SAME noisy, biased, tremor-affected IMU data:
  ORIGINAL : raw gyro counts -> Arduino map() -> platform offset  (legacy code)
  IMPROVED : gyro-bias calibration -> complementary filter -> dead band,
             expo, rate limit, reachability check (pm3rrr_core.imu)
The improved pipeline drives the physics model (MuJoCo, from the URDF).

Run:  python3 simulation/scenarios/s1_imu_teleoperation.py [--no-media]
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
from pm3rrr_sim.runner import arduino_map, run  # noqa: E402

from pm3rrr_core.imu import ComplementaryFilter, Mpu6050Model, TiltTeleop  # noqa: E402
from pm3rrr_core.trajectory import quintic  # noqa: E402

DT_IMU = 0.01
KEYS = [  # (time, roll_deg, pitch_deg): what the operator's hand does
    (0.0, 0, 0), (1.0, 0, 0), (2.2, 0, 25), (3.4, 0, 25), (4.4, -22, 0), (5.4, -22, 0),
    (6.2, 0, 0), (6.6, 0, 0)]


def hand_motion(t):
    """Smooth key-framed tilt + a circular 'stir' + physiological tremor (9 Hz)."""
    if t <= KEYS[-1][0]:
        for (t0, r0, p0), (t1, r1, p1) in zip(KEYS[:-1], KEYS[1:]):
            if t0 <= t <= t1:
                s = quintic(np.array([(t - t0) / (t1 - t0)]))[0][0]
                r, p = r0 + s * (r1 - r0), p0 + s * (p1 - p0)
                break
    elif t <= 10.6:
        u = (t - KEYS[-1][0]) / 4.0
        amp = 22 * np.sin(np.pi * min(u, 1.0)) ** 0.5
        r, p = amp * np.sin(2 * np.pi * u), amp * np.cos(2 * np.pi * u) * (u > 0.02)
        r, p = r, p if u > 0.02 else 0.0
    else:
        r, p = 0.0, 0.0
    tremor = 0.6 * np.sin(2 * np.pi * 9.0 * t)
    return np.radians(r + tremor), np.radians(p + 0.7 * tremor)


def simulate_imu(T):
    """Generate MPU6050 readings and run both pipelines at 100 Hz."""
    imu = Mpu6050Model(seed=7)
    ts = np.arange(0, T + 1e-9, DT_IMU)
    true = np.array([hand_motion(t) for t in ts])
    rates = np.gradient(true, DT_IMU, axis=0)
    cf = ComplementaryFilter(tau=0.4, dt=DT_IMU)
    # power-up calibration: 1 s with the hand still (the operator is told to hold still)
    cf.calibrate([imu.measure(0, 0, np.zeros(3))[1] for _ in range(100)])
    tele = TiltTeleop(max_tilt=np.radians(25), deadband=np.radians(3), expo=0.35,
                      reach=0.016, max_speed=0.05)
    est, new, old = [], [], []
    for k, t in enumerate(ts):
        r, p = true[k]
        acc, gyr = imu.measure(r, p, np.array([rates[k, 0], rates[k, 1], 0.0]))
        er, ep = cf.update(acc, gyr, DT_IMU)
        est.append((er, ep))
        new.append(tele.update(er, ep, DT_IMU))
        # ORIGINAL firmware (parallel_inverse_kinematics.ino): dx = map(gyro_x, -32768, 32768, -10, 12)
        _, graw = Mpu6050Model.to_raw(acc, gyr)
        dx = arduino_map(graw[0], -32768, 32768, -10, 12)
        dy = arduino_map(graw[1], -32768, 32768, -8, 15)
        old.append((dx / 1000.0, dy / 1000.0, 0.0))
    return ts, true, np.array(est), np.array(new), np.array(old)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--no-media", action="store_true")
    a = ap.parse_args()
    T = 12.0
    ts, true, est, new, old = simulate_imu(T)

    def target(t):
        return new[min(int(round(t / DT_IMU)), len(new) - 1)]

    sim = MujocoPm3rrr(width=480, height=400, hand_imu=True)
    sim.set_pose((0, 0, 0))

    fig = plt.figure(figsize=(4.8, 4.0), dpi=100)
    ax1 = fig.add_axes([0.13, 0.62, 0.83, 0.32])
    ax2 = fig.add_axes([0.13, 0.10, 0.83, 0.40])

    def on_frame(k, t, log):
        i = min(int(round(t / DT_IMU)), len(ts) - 1)
        r, p = true[i]
        sim.set_hand(r, p)
        import mujoco
        mujoco.mj_forward(sim.model, sim.data)
        img3d = sim.render(azimuth=145, elevation=-30, distance=0.56, lookat=(0.04, -0.07, 0.08),
                           trail=np.asarray(log.actual)[-150:, :2])
        ax1.cla()
        ax1.plot(ts[:i], np.degrees(true[:i, 1]), color="#1f77b4", lw=1, alpha=0.5, label="pitch (true)")
        ax1.plot(ts[:i], np.degrees(est[:i, 1]), color="#1f77b4", lw=1.6, label="pitch (filtered)")
        ax1.plot(ts[:i], np.degrees(true[:i, 0]), color="#2ca02c", lw=1, alpha=0.5, label="roll (true)")
        ax1.plot(ts[:i], np.degrees(est[:i, 0]), color="#2ca02c", lw=1.6, label="roll (filtered)")
        ax1.set_xlim(0, T)
        ax1.set_ylim(-30, 30)
        ax1.set_ylabel("hand tilt [deg]", fontsize=8)
        ax1.tick_params(labelsize=7)
        ax1.legend(fontsize=6, ncol=2, loc="lower left")
        ax1.set_title("S1  IMU gesture teleoperation", fontsize=10, weight="bold")
        ax2.cla()
        setup_topview(ax2, lim=22)
        ax2.add_patch(plt.Circle((0, 0), 16, fill=False, ec="#bbb", ls=":"))
        ax2.plot(old[:i, 0] * 1e3, old[:i, 1] * 1e3, ".", color="#777", ms=3,
                 label="original: raw gyro + map()")
        act = np.asarray(log.actual)
        ax2.plot(act[:, 0] * 1e3, act[:, 1] * 1e3, "-", color="#d62728", lw=1.8,
                 label="improved: filter + teleop (physics)")
        ax2.plot(act[-1, 0] * 1e3, act[-1, 1] * 1e3, "o", color="#d62728")
        ax2.legend(fontsize=6, loc="upper left")
        ax2.tick_params(labelsize=7)
        ax2.xaxis.label.set_size(8)
        ax2.yaxis.label.set_size(8)
        ax2.text(0.98, 0.03, f"t = {t:4.1f} s", transform=ax2.transAxes, ha="right", fontsize=8)
        return hstack(img3d, fig_to_array(fig))

    log, frames = run(sim, target, T, dt_ctrl=0.02, frame_every=4,
                      on_frame=None if a.no_media else on_frame)
    L = log.arrays()
    err = np.linalg.norm(L["actual"][:, :2] - L["target"][:, :2], axis=1)
    rest = (ts >= 0.2) & (ts < 1.0)
    jit_new = np.std(new[rest, :2] * 1e3, axis=0).max()
    jit_old = np.std(old[rest, :2] * 1e3, axis=0).max()
    hold = (ts > 2.8) & (ts < 3.4)        # hand held at 25 deg pitch
    print("Scenario 1 - IMU teleoperation")
    print(f"  tilt estimate RMS error        : {np.degrees(np.sqrt(np.mean((est - true) ** 2))):.2f} deg")
    print(f"  platform motion while hand HELD tilted: improved {1e3*new[hold, 0].mean():5.1f} mm, "
          f"original {1e3*old[hold, 0].mean():5.1f} mm  (original only reacts to rotation RATE)")
    print(f"  jitter at rest (std)           : improved {jit_new:.3f} mm, original {jit_old:.3f} mm")
    print(f"  physics tracking error (RMS)   : {1e3*np.sqrt(np.mean(err**2)):.3f} mm, max {1e3*err.max():.3f} mm")
    print(f"  peak servo torque              : {np.abs(L['tau']).max()*100:.1f} N cm")
    if not a.no_media:
        save_gif(frames, MEDIA / "scenarios" / "s1_imu_teleoperation.gif", fps=12)
        save_mp4(frames, MEDIA / "videos" / "s1_imu_teleoperation.mp4", fps=12)


if __name__ == "__main__":
    main()
