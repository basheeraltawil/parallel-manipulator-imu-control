# S5 · Singularity protection: "rotate the part by 40°"

![S5](../media/scenarios/s5_singularity_guard.gif)

## Problem

An operator, or a CAM program, asks the platform to rotate a part from 0° to **−40°**. Both end poses are reachable and well conditioned. But the straight path crosses a **Type-II singularity** at −25.4° (κ = 338), and the goal lies in **another assembly mode** ([lessons 05–06](../learning/06_jacobian_singularities.md)). A 0.28 N process force acts on the platform, as some force always does.

## What happens

| | Unguarded | Guarded (`guard_trajectory`, κ ≤ 10) |
|---|---|---|
| Motor angles at the end | exactly the goal's (error 0.05°) | stopped at the last safe pose |
| Platform orientation at the end | **−12.9°** (goal −40°): **27° error, undetected** | −15.9° (safe stop at −22.4°; the rest is servo dead band, see below) |
| Controller's belief | "at −40°" (wrong!) | **fault raised**: "condition number above 10 (near a parallel singularity)" |

Why −12.9°? The forward kinematics of the goal's motor angles has two solutions: φ = −40.0° and **φ = −10.4°**. While crossing the singularity the platform lost stiffness and, pushed by the process force, stayed in the original assembly mode. The servos did their job perfectly; the *robot* did not.

The guarded robot lags its stop pose by about 6.5°. This is the hobby servos' 0.5° dead band amplified by the poor orientation resolution near the singular region (κ ≈ 8). With ideal servos the error is 0.04°, with smart servos 0.35°. This is one more reason to keep a **margin** from singularities.

## Lessons

1. Checking the end points is not enough: **check every sample of the path** (κ, IK, sign of $\det\mathbf{J}_x$).
2. Without position sensing on the platform, an assembly-mode change is **invisible** to the controller. The guard must prevent it.
3. A goal in another assembly mode is unreachable in practice. Re-grip the part (rotate twice by 20°), or redesign (a larger orientation range needs a different geometry or redundant actuation).
4. Industrial controllers do exactly this ("path rejected: singularity"). The same guard runs in `controller_node.py` (κ ≤ 10, hold the last safe pose) and in the firmware (cheap Type-I/II thresholds).

## Run it

```bash
python3 simulation/scenarios/s5_singularity_guard.py
ros2 launch pm3rrr_bringup sim_rviz.launch.py scenario:=rotation_limit     # watch /pm3rrr/status
```
