# S1 · Gesture (IMU) teleoperation

![S1](../media/scenarios/s1_imu_teleoperation.gif)

## Problem

An operator moves a small stage by **tilting a hand-held MPU6050**: forward/back → x, left/right → y. This was the original goal of the project ("gyroscope trajectory generation"). Uses: positioning a microscope or camera, micro-manipulation, a rehabilitation or teaching device where the user "feels" the motion.

Requirements: the platform follows the **tilt angle**, holds still when the hand is still, ignores hand tremor, never leaves the safe workspace, and moves smoothly.

## Setup

* The hand performs: 1 s still (calibration) → pitch 25° → roll −22° → a circular "stir" → rest, plus a **9 Hz physiological tremor** (0.6°).
* IMU model: MPU6050 noise, gyro bias (0.8, −0.5, 0.3 °/s), accelerometer bias, int16 quantisation.
* Two pipelines on the same data:
  * **Original:** `dx = map(gyro_x, -32768, 32768, -10, 12)` (exact Arduino integer `map()`).
  * **Improved:** gyro-bias calibration → complementary filter (τ = 0.4 s) → dead band 3° → expo 0.35 → reach 16 mm → rate limit 50 mm/s → reachability check.
* The improved targets drive the physics model with `writeMicroseconds` servos.

## Results

| KPI | Original | Improved |
|---|---|---|
| Platform motion while the hand is **held** at 25° | 0.5 mm (reacts only to rotation *rate*) | **15.4 mm** (follows the angle) |
| Jitter at rest (std) | 1.13 mm | **0.000 mm** (dead band) |
| Tilt estimate RMS error | — | 0.29° |
| Physics tracking error (RMS / max) | — | 0.79 / 2.06 mm (servo lag + dead band) |
| Peak servo torque | — | 0.002 N m (≪ 1.08 N m) |

## Lessons

1. Map **angles**, not rates, to positions, and fuse gyro and accelerometer to get the angle ([lesson 10](../learning/10_imu_sensor_fusion.md)).
2. Human interfaces need a **dead band, expo and rate limit**, as every industrial joystick does.
3. Always run the target through the IK validity check before commanding it.

## Run it

```bash
python3 simulation/scenarios/s1_imu_teleoperation.py
ros2 launch pm3rrr_bringup sim_rviz.launch.py scenario:=imu          # simulated hand IMU in RViz
ros2 launch pm3rrr_bringup hardware.launch.py scenario:=imu           # real MPU6050 via the Arduino
```

The firmware also does this **stand-alone** (mode `M 1`, the default when the IMU is found), without a PC.
