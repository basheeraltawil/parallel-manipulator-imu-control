# 05 · Calibration and commissioning

A robot is only as accurate as its calibration. Do these steps in order and record the results in a **commissioning record** (template at the end).

## Step 1: Power and wiring check
`firmware/bringup/servo_sweep`: each servo sweeps ±30° smoothly, without resets or jitter. If the Arduino resets, the supply or ground is wrong (chapter 03).

## Step 2: IMU check
`firmware/bringup/imu_check`: flat and still gives az ≈ +9.8 m/s², gyro ≈ 0 ± 1 °/s, roll and pitch ≈ 0. Tilt nose-down → **pitch positive**. If an axis is inverted, fix the mounting or the sign in `imu.cpp`.

## Step 3: Servo zero (trim)
`firmware/bringup/servo_calibration`, with only the proximal links mounted:

1. Print the alignment jig or use a protractor centred on the servo axis.
2. For servo *i*, send `i 1500` and adjust the pulse until the link points along its **home direction** $\theta_i^{home}$ (94.29°, −145.71°, −25.71° in the base frame; lesson 02).
3. Write `trim = pulse − 1500` into `SERVO_TRIM_US[i]` in `config.h`.
4. Check the **direction**: +100 µs must rotate the link counter-clockwise seen from above; otherwise set `SERVO_DIRECTION[i] = -1`.
5. Check the pulse range: verify that 1000/2000 µs really give 0/180° on *your* servos, then adjust `SERVO_US_MIN/MAX`.

## Step 4: Kinematic calibration (identification)
Real link lengths and joint positions differ from CAD (lesson 02: the proximal link measures 49.35 mm, not 50). Procedure:

1. Command N ≥ 20 poses spread over the workspace.
2. Measure the real platform pose: a camera looking down at an ArUco marker on the platform, or a dial gauge or calipers against fixed references.
3. Identify the parameters $\boldsymbol\pi = (l_{1,i}, l_{2,i}, \theta^{home}_i, \ldots)$ by least squares:

$$ \min_{\boldsymbol\pi} \sum_k \big\| \mathbf{x}^{meas}_k - \mathrm{FK}(\boldsymbol\theta_k;\boldsymbol\pi) \big\|^2 $$

(use `Pm3rrrKinematics(RobotParams(...)).fk` inside `scipy.optimize.least_squares`).

4. Write the identified values into `robot_params.yaml` and `config.h`, then re-run the tests.

## Step 5: Functional tests
| Test | Pass criterion |
|---|---|
| Home on boot | all servos at 90° ± trim, platform centred |
| `P 10 0 0`, `P 0 10 0`, `P 0 0 10` | the platform moves +x, +y, CCW |
| `P 60 0 0` | rejected (`E ... unreachable`), no motion |
| `P 0 0 -40` | rejected (`E ... near singularity`) |
| Watchdog | stream stops → hold within 0.5 s |
| E-stop | servo power off, Arduino stays on, restart needs a deliberate action |
| IMU teleop | tilt 25° → about 15 mm motion, still hand → no motion |

## Commissioning record (template)

```
Robot serial: ______  Date: ______  Engineer: ______
Firmware version/commit: ______
Servo trims [us]: s1 ___  s2 ___  s3 ___   Directions: ___ ___ ___
Identified l1 [mm]: ___ ___ ___   l2 [mm]: ___ ___ ___
Gyro bias at boot [deg/s]: ___ ___ ___
Functional tests: home □  axes □  reject-unreachable □  reject-singular □  watchdog □  e-stop □  teleop □
Accuracy/repeatability (chapter 06): ______
```
