# Firmware

| Folder | What | When |
|---|---|---|
| `pm3rrr_firmware/` | Main firmware: IK + singularity guard, MPU6050 fusion, teleoperation, serial protocol, watchdog | always |
| `bringup/servo_sweep/` | Servo power/wiring check | commissioning step 1 |
| `bringup/imu_check/` | MPU6050 wiring/axes check | commissioning step 2 |
| `bringup/servo_calibration/` | Find the per-servo trim (`SERVO_TRIM_US`) | commissioning step 3 |
| `test/` | PC unit test of the firmware's C++ logic vs. the Python library | CI / before flashing |

```bash
make -C firmware/test            # host test (g++ + python3 numpy)
arduino-cli compile --fqbn arduino:avr:uno firmware/pm3rrr_firmware
arduino-cli upload  --fqbn arduino:avr:uno -p /dev/ttyUSB0 firmware/pm3rrr_firmware
```

Build size on an Arduino Uno: **19.2 kB flash (58 %), 752 B RAM (37 %)**.

Modes: `M 0` stream from PC/ROS 2, `M 1` stand-alone IMU teleoperation (default when the hand IMU is found), `M 2` hold.
Protocol and commissioning: [`docs/industrialization/04_firmware_and_protocol.md`](../docs/industrialization/04_firmware_and_protocol.md).
