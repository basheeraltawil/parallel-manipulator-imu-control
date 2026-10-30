# Deploy configuration

| Item | Value | Notes |
|---|---|---|
| Default board | Arduino Uno | FQBN `arduino:avr:uno` |
| Alternatives | Nano `arduino:avr:nano:cpu=atmega328`, Mega `arduino:avr:mega`, ESP32 `esp32:esp32:esp32` | ESP32: change the pins in `config.h`, use `ESP32Servo` |
| Serial port | `/dev/ttyUSB0` (CH340 Nano) · `/dev/ttyACM0` (Uno) | overridden by `PM3RRR_SERIAL_PORT` in `.claude/settings.local.json` |
| Baud rate | 115200 | must match `serial_bridge` parameter `baud` |
| ROS distro | Humble (Ubuntu 22.04) | Gazebo Classic 11 |
| Scenarios | `figure8`, `microplate`, `contour`, `rotation_limit`, `imu`, `none` | |
| Firmware size budget | flash < 80 %, RAM < 60 % on Uno | currently 58 % / 37 % |
