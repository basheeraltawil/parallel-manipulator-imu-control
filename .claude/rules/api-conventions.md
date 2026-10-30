---
paths:
  - "ros2_ws/src/pm3rrr_control/**"
  - "ros2_ws/src/pm3rrr_bringup/**"
  - "firmware/pm3rrr_firmware/protocol.*"
  - "firmware/pm3rrr_firmware/pm3rrr_firmware.ino"
---
# Interface conventions (ROS 2 topics and the serial protocol)

## ROS 2
- Namespace `/pm3rrr/...` for robot-specific topics; standard names for standard data (`/joint_states`, `/robot_description`).
- Units: REP 103 (m, rad, s). Frames: REP 105 style; fixed frame `world`, tool frame `tool0`, platform centre `platform_center`.
- Commands: `/pm3rrr/target_pose` (`geometry_msgs/Pose2D`). Diagnostics: `/pm3rrr/status` (`std_msgs/String`, human-readable, includes κ and the fault).
- Every new topic must be added to the table in `docs/learning/12_ros2_architecture.md`.
- Parameters in `pm3rrr_control/config/controller.yaml`, keyed by node name.

## Serial protocol (PC ↔ firmware)
- ASCII, one command per line, `\n`-terminated, 115200 baud, first character = command letter.
- PC→MCU: `J`, `P`, `H`, `M`, `C`, `?`. MCU→PC: `I`, `S`, `OK`, `E`.
- Adding a command means changing **both** `serial_bridge_node.py` and `protocol.cpp`, extending both unit tests, and updating `docs/industrialization/04_firmware_and_protocol.md`.
- The firmware validates every command and answers invalid ones with `E <reason>`; it never moves on an invalid command.
