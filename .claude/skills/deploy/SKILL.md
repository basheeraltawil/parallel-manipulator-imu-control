---
name: deploy
description: Build, test and deploy the robot software - flash the Arduino firmware and/or build the ROS 2 Humble workspace and bring up simulation or hardware. Use when the user asks to deploy, flash, upload, build the workspace, or run the robot.
---
# Deploy the pm3rrr robot

Read `deploy-config.md` (next to this file) for boards, ports and targets.

## 0. Gate: tests must pass first
```bash
python3 -m pytest -q ros2_ws/src/pm3rrr_core/test
python3 tools/cad_to_urdf/step4_validate_urdf.py
make -C firmware/test
```
Stop and report if anything fails. Never deploy red.

## 1. Firmware → Arduino
```bash
arduino-cli core install arduino:avr                       # once
arduino-cli compile --fqbn <FQBN> firmware/pm3rrr_firmware
arduino-cli upload  --fqbn <FQBN> -p <PORT> firmware/pm3rrr_firmware
```
After the upload, open the serial monitor (115200) and expect `OK ready`. If the IMU is missing, the firmware reports `E hand IMU not found` and starts in STREAM mode.
**Always ask the user before uploading:** the robot will move to home.

## 2. ROS 2 workspace
```bash
source /opt/ros/humble/setup.bash
cd ros2_ws && rosdep install --from-paths src -y --ignore-src
colcon build --symlink-install && source install/setup.bash
colcon test && colcon test-result --verbose
```

## 3. Bring-up targets
| Target | Command |
|---|---|
| sim-rviz | `ros2 launch pm3rrr_bringup sim_rviz.launch.py scenario:=<scenario>` |
| sim-gazebo | `ros2 launch pm3rrr_bringup sim_gazebo.launch.py scenario:=<scenario> gui:=<true/false>` |
| hardware | `ros2 launch pm3rrr_bringup hardware.launch.py port:=<PORT> scenario:=<scenario>` |

For hardware: confirm the e-stop is reachable, then check `/pm3rrr/status` shows `OK`.

## 4. Report
Report the versions (git commit), what was flashed or built, the test results, and any warnings.
