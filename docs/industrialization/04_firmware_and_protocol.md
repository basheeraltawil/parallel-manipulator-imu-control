# 04 · Firmware and communication protocol

## Firmware structure (`firmware/pm3rrr_firmware`)

| File | Responsibility |
|---|---|
| `config.h` | **the only file to edit**: pins, geometry (= `robot_params.yaml`), servo centre/direction/trim, limits, timing, teleop tuning |
| `kinematics.cpp` | closed-form IK, servo mapping, reachability, travel and singularity (Type-I/II) checks. Unit-tested on the PC against Python (max difference 0.00016°) |
| `imu.cpp` | MPU6050 over raw I²C (DLPF 44 Hz, ±2 g, ±250 °/s), gyro calibration, complementary filter |
| `protocol.cpp` | line parser (unit-tested) |
| `pm3rrr_firmware.ino` | non-blocking scheduler: IMU 100 Hz, control 50 Hz, status 10 Hz; modes, watchdog, slew limit |

```bash
make -C firmware/test                                                  # host unit tests
arduino-cli compile --fqbn arduino:avr:uno firmware/pm3rrr_firmware
arduino-cli upload  --fqbn arduino:avr:uno -p /dev/ttyUSB0 firmware/pm3rrr_firmware
```

## Modes

| Mode | Command | Behaviour |
|---|---|---|
| 0 STREAM | `M 0` | follows `J`/`P` lines from the PC; **watchdog 0.5 s** → hold if the PC goes silent |
| 1 IMU | `M 1` | stand-alone teleoperation from the hand IMU (default at boot when the IMU is found) |
| 2 HOLD | `M 2` | holds the last output |

## Protocol (ASCII, 115200 baud, `\n`-terminated)

| Direction | Line | Meaning |
|---|---|---|
| PC → MCU | `J s1 s2 s3` | servo angles in degrees (0–180, 90 = centre), checked against ±85° |
| PC → MCU | `P x_mm y_mm phi_deg` | pose; IK + all checks run on the MCU |
| PC → MCU | `H` / `C` / `?` / `M n` | home / calibrate gyro / info / mode |
| MCU → PC | `I ax ay az gx gy gz` | hand IMU, m/s² and rad/s (bias removed), 100 Hz |
| MCU → PC | `S s1 s2 s3 mode` | servo outputs (deg), 10 Hz |
| MCU → PC | `OK text` / `E text` | acknowledgement / error ("rejected pose: near singularity") |

Why ASCII? It is human-readable (`screen /dev/ttyUSB0 115200`), easy to log and to teach. For a product, switch to a binary framed protocol with CRC (e.g. COBS + CRC16), or to **micro-ROS** on an ESP32/Teensy, which publishes ROS 2 topics directly from the MCU.

## ROS 2 integration

* Today: `pm3rrr_control/serial_bridge` ↔ firmware (the topics of lesson 12).
* Product path: a C++ `hardware_interface::SystemInterface` plugin (`read()` = parse `S` lines or read the smart-servo positions, `write()` = send `J`). Then the Gazebo controllers (`joint_trajectory_controller`) run unchanged on hardware.

## Robustness rules applied in the firmware

* No `delay()` in the loop. No `String` (heap fragmentation on AVR).
* Validate every command: range, reachability, singularity. Reject with `E` and keep the last safe output.
* Slew-rate limit on the servo outputs (300 °/s), so there are no jumps after a bad command.
* Watchdog in stream mode.
* `writeMicroseconds` with per-servo trim.

## Checklist

- [ ] `make -C firmware/test` passes
- [ ] `config.h` geometry equals `robot_params.yaml`
- [ ] Watchdog tested (unplug USB during a stream → robot holds)
- [ ] Rejected-command path tested (`P 60 0 0` → `E rejected pose: unreachable`)
