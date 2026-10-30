# CLAUDE.local.md: personal overrides (template)

This file is for **your** machine-specific notes. Claude Code loads it after `CLAUDE.md`.
Edit it locally; keep secrets out (use environment variables).

## My environment (example; edit me)

- ROS 2 Humble sourced from `/opt/ros/humble/setup.bash`
- Robot connected at `/dev/ttyUSB0` (Arduino Uno), servo supply 6 V / 5 A
- Gazebo GUI: on (`gui:=true`); set `false` on headless machines

## My preferences (example)

- Explain maths changes with a short derivation before editing code.
- Run `make -C firmware/test` before any firmware upload.
