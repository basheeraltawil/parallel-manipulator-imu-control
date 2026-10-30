# Legacy code (original experiments, kept for reference)

These are the original sketches and scripts, **unchanged**, so you can compare them with the new implementation and learn from the review. Do not flash them on the robot: use [`firmware/pm3rrr_firmware`](../firmware/pm3rrr_firmware) instead.

## File-by-file review

| Original file | What it did | Issues found | Replaced by |
|---|---|---|---|
| `codes/servo/servo.ino` | sweeps one servo 0→150° | blocking `delay()` | `firmware/bringup/servo_sweep` |
| `codes/accelerometer/accelerometer.ino` | prints raw/normalised acceleration (Jarzebski library) | `mpu.start(...)` is not part of that library's API (it is `begin()`) | `firmware/bringup/imu_check` |
| `codes/aceleo_gyro/aceleo_gyro.ino` | first "IMU drives a servo" test: accel Y → servo angle | `zs` computed from `accelerometer_y` (copy-paste); raw counts through integer `map()`; no filtering | `firmware/pm3rrr_firmware/imu.cpp` |
| `codes/gyro/servo_mpu6050/servo_mpu6050.ino` | reads **two** MPU6050s (0x68 and 0x69) | `Wire.write(0x00000000)` written as hex where binary was meant (works by luck: both are 0); busy-wait `while(Wire.available()<6)` can hang | dual-IMU support in `config.h` (`IMU_AUX_ADDR`) |
| `codes/first_manu/first_manu.ino` | IK of one leg, sweeps the servo 0→θ once | θ can exceed 180°, so the servo saturates; no servo-centre offset | `kinematics.cpp` |
| `codes/inverse_kin/inverse_kin.ino` | accel → (px, py) of one leg → IK → servo | only one leg; θ = α+β without wrapping or limits | `kinematics.cpp`, `teleopStep()` |
| `codes/parallel_inverse_kinematics/parallel_inverse_kinematics.ino` | **main version**: gyro → platform offset (dx, dy) → IK of 3 legs → 3 servos | ① gyro **rate** mapped to **position** (S1); ② `map()` is integer, so 1 mm steps; ③ in the `else` branch `tests.write(theta)` runs **before** θ is computed (uninitialised value); ④ angles 0…360° sent to 0…180° servos; ⑤ no reachability/singularity check; ⑥ `delay()` per servo makes the loop slow and uneven | the whole new firmware |
| `codes/organised_inverse/organised_inverse.ino` | refactoring attempt into functions | **does not compile**: `px1, xp, dx, a, h, yR…` undeclared, missing `;` after `third_manio_inv()`, `loop()` empty | `pm3rrr_firmware.ino` structure |
| `codes/inversetrying/inversetrying.ino` | 3-leg IK with a fixed displacement, sweeps 0→θ every loop | re-sweeps from 0 forever; `theta2` uninitialised in one branch; servo 3 prints "servo2" | — |
| `codes/singular_manipulators/manipulator1..3` | serial jog: type a relative angle, the servo steps there | positive jogs end 1° short (the second `while` runs once with `state == 0`); `String` on AVR | `servo_calibration` sketch |
| `codes/inverse_kinematics.py`, `inverse_trying.py` | one-leg IK check with fixed numbers | branch logic duplicated and inconsistent with the sketches | `pm3rrr_core.kinematics` |
| `codes/parallel_manipulator_analyse.py` | home-pose IK of the 3 legs | uses platform joints on a **15 mm** circle (offsets 13 / 7.5 / 15) while the CAD platform has **26 mm** (the `r = 26` variable is defined but unused) | `robot_params.yaml` (26 mm, measured) |

## What the original got right (and was kept)

* The **closed-form IK** via the isosceles triangle ($a = d/2$, $h=\sqrt{L^2-a^2}$, $\theta=\beta\pm\alpha$): generalised to the law of cosines.
* The **working-mode** idea ($\theta = \beta \pm \alpha$) and the choice $\theta=\beta-\alpha$, which is the best-conditioned mode.
* The geometry: base radius 87.5 mm and footprint values ($x_p$ = 76, $y_p$ = 44), confirmed by the CAD.
* Two MPU6050s on one bus (0x68/0x69).
* The concept of steering the platform with a hand-held IMU, now scenario S1 and firmware mode 1.
