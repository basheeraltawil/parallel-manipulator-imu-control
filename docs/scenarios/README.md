# Scenarios: real problems, simulated

Each scenario is a real industrial or educational task, simulated with **physics** (MuJoCo, built from the same URDF as ROS) and a **realistic hobby-servo model** (quantisation, 0.5° dead band, speed limit, 40 ms lag). Every number below is printed by the scripts.

| # | Problem | Industry analogue | Key result | Script |
|---|---|---|---|---|
| [S1](s1_imu_teleoperation.md) | Steer the platform by tilting a hand-held IMU | micro-manipulation, camera positioning, rehab/teaching devices | original pipeline: 0.5 mm motion when the hand is held at 25°; improved: **15.4 mm**, no jitter at rest | `s1_imu_teleoperation.py` |
| [S2](s2_precision_pick_place.md) | Stop precisely over the wells of a micro-plate | lab automation, inspection, pick & place | stop error 0.66 → 0.56 mm (µs commands) → **0.06 mm** (smart servo) | `s2_precision_pick_place.py` |
| [S3](s3_contour_dispensing.md) | Dispense a glue bead around a part at constant speed | dispensing, laser engraving, cutting | fastest feed within ±0.5 mm: **40 mm/s** | `s3_contour_dispensing.py` |
| [S4](s4_base_motion_compensation.md) | Keep a tool on a world-fixed target while the base rotates | stabilisation on vehicles/ships, hand-held tools | position error 0.85 → **0.20 mm**, rotation 6.8° → **1.0°** | `s4_base_motion_compensation.py` |
| [S5](s5_singularity_guard.md) | "Rotate the part by 40°" through a singularity | safety / motion-planning validation | unguarded: platform **27° off** while the motors look right; guard: safe stop at −22° | `s5_singularity_guard.py` |

```bash
python3 simulation/run_all.py              # all scenarios + GIFs + MP4s (~10 min)
python3 simulation/run_all.py --no-media   # numbers only (~2 min)
```

The same motions run in ROS 2: `ros2 launch pm3rrr_bringup sim_rviz.launch.py scenario:=<figure8|microplate|contour|rotation_limit|imu>` (or `sim_gazebo.launch.py`).
