# S2 · Precision positioning over a micro-plate

![S2](../media/scenarios/s2_precision_pick_place.gif)

## Problem

A lab-automation stage must stop over the wells of a standard micro-plate (**9 mm pitch**, SBS/ANSI) so that a pipette, camera or probe can work. KPIs: **accuracy at each stop** and **throughput**.

## Setup

* 6 wells (2 × 3, centred), sequence home → W1…W6 → home.
* Rest-to-rest **quintic** moves, $v_{max}$ = 60 mm/s, $a_{max}$ = 0.6 m/s², 0.5 s dwell. The error is measured at the end of each dwell, when the pipette would act.
* Three actuator options in physics:
  1. hobby servo + `Servo.write(int deg)` (original firmware): 1° steps;
  2. hobby servo + `writeMicroseconds()` (new firmware): 0.18° steps;
  3. smart servo with a 12-bit encoder (Dynamixel class): 0.088°, 0.05° dead band.

## Results

| Option | Mean stop error | Max stop error |
|---|---|---|
| hobby, `write(deg)` | 0.66 mm | 0.81 mm |
| hobby, `writeMicroseconds` | 0.56 mm | 0.62 mm |
| **smart servo** | **0.06 mm** | **0.07 mm** |

Cycle time: 5.6 s for 6 wells (64 wells/min including dwell). Peak torque: 0.003 N m. The motors are nowhere near their torque limit; **accuracy, not force, is the constraint**.

## Lessons

1. Finer command resolution helps only a little, because the servo's **dead band** (≈0.5°) dominates. The Jacobian turns 0.5° into ≈0.57 mm at home ([accuracy map](../learning/11_control_and_actuators.md)).
2. For sub-0.1 mm work you need actuators with **position feedback**. That is the first item on the [industrialization BOM](../industrialization/02_bill_of_materials.md).
3. With quintic moves the platform arrives without overshoot, so the dwell can be short.

## Run it

```bash
python3 simulation/scenarios/s2_precision_pick_place.py
ros2 launch pm3rrr_bringup sim_gazebo.launch.py scenario:=microplate
```
