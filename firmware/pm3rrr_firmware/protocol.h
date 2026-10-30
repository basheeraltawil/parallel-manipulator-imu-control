// Line protocol PC <-> firmware (see serial_bridge_node.py and
// docs/industrialization/04_firmware_and_protocol.md). Pure C++, unit-tested on the PC.
#pragma once

enum CmdType { CMD_NONE, CMD_JOINTS, CMD_POSE, CMD_HOME, CMD_MODE, CMD_CALIBRATE, CMD_INFO, CMD_ERROR };

struct Command {
  CmdType type = CMD_NONE;
  float v[3] = {0, 0, 0};
  int mode = 0;
};

// Parse one '\0'-terminated line (without the newline).
Command parse_command(const char *line);
