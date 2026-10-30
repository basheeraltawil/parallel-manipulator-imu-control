#!/usr/bin/env python3
"""Scenario 4 - Stabilising a tool while the robot's base rotates.

Real problem: the robot is mounted on something that moves - a vehicle,
a ship deck, a rotating fixture, a hand-held device. A sensor/laser/camera on
the platform must stay on a target that is fixed in the WORLD. An MPU6050 on
the base measures the base yaw rate; the controller integrates it and
commands the counter-motion  x_base = R(-psi_hat) (x_world)  ,  phi_base = phi_world - psi_hat.

Compared:
  OFF : no compensation (platform moves with the base)
  ON  : gyro-based compensation with power-up bias calibration and a small
        prediction (psi_hat + psi_dot * T_lag) that cancels the servo lag

Rendering trick: physics runs in the base frame; the camera is counter-rotated
by -psi, so the video shows what a world-fixed observer sees.

Run:  python3 simulation/scenarios/s4_base_motion_compensation.py [--no-media]
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
from pm3rrr_sim.runner import run  # noqa: E402

from pm3rrr_core import rot2  # noqa: E402
from pm3rrr_core.imu import Mpu6050Model  # noqa: E402

TARGET_W = np.array([0.006, 0.004, 0.0])      # world-fixed aim point [m, m, rad]
T = 8.0
DT_IMU = 0.005


def base_yaw(t):
    """Vehicle/deck yaw: two sinusoids, ramped in (deg)."""
    ramp = min(1.0, t / 1.0)
    return np.radians(ramp * (9.0 * np.sin(2 * np.pi * 0.35 * t) + 3.0 * np.sin(2 * np.pi * 1.1 * t + 0.5)))


def estimate_yaw():
    """Integrate the base gyro (z) after a 1 s bias calibration at rest."""
    imu = Mpu6050Model(seed=11, gyro_bias=np.radians([0.0, 0.0, 1.2]))
    calib = np.mean([imu.measure(0, 0, np.zeros(3))[1][2] for _ in range(200)])
    ts = np.arange(0, T + 1e-9, DT_IMU)
    psi = np.array([base_yaw(t) for t in ts])
    rate = np.gradient(psi, DT_IMU)
    est = np.zeros_like(ts)
    for k in range(1, len(ts)):
        g = imu.measure(0, 0, np.array([0, 0, rate[k]]))[1][2] - calib
        est[k] = est[k - 1] + g * DT_IMU
    rate_est = np.gradient(est, DT_IMU)
    return ts, psi, est, rate_est


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--no-media", action="store_true")
    a = ap.parse_args()
    ts, psi, psi_hat, rate_hat = estimate_yaw()
    T_LAG = 0.045                                  # servo lag to cancel (first-order model)

    def idx(t):
        return min(int(round(t / DT_IMU)), len(ts) - 1)

    def target_on(t):
        k = idx(t)
        ps = psi_hat[k] + rate_hat[k] * T_LAG
        xy = rot2(-ps) @ TARGET_W[:2]
        return np.array([xy[0], xy[1], TARGET_W[2] - ps])

    def target_off(t):
        return TARGET_W

    def world_error(L):
        e_pos, e_rot = [], []
        for t, x in zip(L["t"], L["actual"]):
            ps = psi[idx(t)]
            pw = rot2(ps) @ x[:2]
            e_pos.append(np.linalg.norm(pw - TARGET_W[:2]))
            e_rot.append(np.degrees(x[2] + ps - TARGET_W[2]))
        return np.array(e_pos), np.array(e_rot)

    sim_off = MujocoPm3rrr(width=480, height=400)
    sim_off.set_pose(TARGET_W)
    log_off, _ = run(sim_off, target_off, T, servo_mode="us")
    e_off = world_error(log_off.arrays())

    sim = MujocoPm3rrr(width=480, height=400)
    sim.set_pose(TARGET_W)
    fig = plt.figure(figsize=(4.8, 4.0), dpi=100)
    ax1 = fig.add_axes([0.14, 0.60, 0.82, 0.32])
    ax2 = fig.add_axes([0.14, 0.10, 0.82, 0.36])
    z_tool = 0.0808 + 0.016 + 0.0075 + 0.001
    Loff = log_off.arrays()

    def on_frame(k, t, log):
        ps = psi[idx(t)]
        tgt_sim = rot2(-ps) @ TARGET_W[:2]            # world target expressed in the base frame
        img = sim.render(azimuth=90 - np.degrees(ps), elevation=-62, distance=0.36, lookat=(0, 0, 0.07),
                         markers=[((tgt_sim[0], tgt_sim[1], z_tool), (0.1, 0.9, 0.2, 1), 0.0025)])
        L = log.arrays()
        ep, er = world_error(L)
        n = len(ep)
        ax1.cla()
        ax1.plot(ts, np.degrees(psi), color="#555", lw=1, label="base yaw (true)")
        ax1.plot(ts[:idx(t)], np.degrees(psi_hat[:idx(t)]), color="#ff7f0e", lw=1.6, label="gyro estimate")
        ax1.set_xlim(0, T)
        ax1.set_ylim(-14, 14)
        ax1.set_ylabel("yaw [deg]", fontsize=7)
        ax1.legend(fontsize=6, loc="lower left", ncol=2)
        ax1.tick_params(labelsize=7)
        ax1.set_title("S4  Base-motion compensation", fontsize=10, weight="bold")
        ax2.cla()
        ax2.plot(Loff["t"][:n], e_off[0][:n] * 1e3, color="#999", lw=1.2, label="position err, OFF")
        ax2.plot(L["t"], ep * 1e3, color="#2ca02c", lw=1.8, label="position err, ON")
        ax2.plot(Loff["t"][:n], np.abs(e_off[1][:n]) / 10, color="#bbb", lw=1, ls="--", label="|rot err|/10 [deg], OFF")
        ax2.plot(L["t"], np.abs(er) / 10, color="#1f77b4", lw=1.2, ls="--", label="|rot err|/10 [deg], ON")
        ax2.set_xlim(0, T)
        ax2.set_ylim(0, 2.0)
        ax2.set_xlabel("t [s]", fontsize=7)
        ax2.set_ylabel("world-frame error [mm]", fontsize=7)
        ax2.legend(fontsize=5.5, loc="upper left", ncol=2)
        ax2.tick_params(labelsize=7)
        return hstack(img, fig_to_array(fig))

    log_on, frames = run(sim, target_on, T, servo_mode="us", frame_every=5,
                         on_frame=None if a.no_media else on_frame)
    e_on = world_error(log_on.arrays())
    s = np.asarray(log_on.t) > 1.0
    print("Scenario 4 - base-motion compensation (base yaw up to +-12 deg)")
    print(f"  yaw estimate drift after {T:.0f} s     : {np.degrees(abs(psi_hat[-1] - psi[-1])):.2f} deg")
    for name, (ep, er) in (("OFF", e_off), ("ON ", e_on)):
        print(f"  compensation {name}: world position error RMS {1e3*np.sqrt(np.mean(ep[s]**2)):.2f} mm "
              f"(max {1e3*ep[s].max():.2f}),  orientation error RMS {np.sqrt(np.mean(er[s]**2)):.2f} deg")
    if frames:
        save_gif(frames, MEDIA / "scenarios" / "s4_base_motion_compensation.gif", fps=10, max_colors=128, scale=0.85)
        save_mp4(frames, MEDIA / "videos" / "s4_base_motion_compensation.mp4", fps=10)


if __name__ == "__main__":
    main()
