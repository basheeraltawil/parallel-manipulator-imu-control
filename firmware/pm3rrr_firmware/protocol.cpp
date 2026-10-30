#include "protocol.h"

#include <stdlib.h>

static const char *skip(const char *p) {
  while (*p == ' ' || *p == '\t') ++p;
  return p;
}

static int read_floats(const char *p, float *out, int n) {
  int got = 0;
  for (; got < n; ++got) {
    p = skip(p);
    char *end;
    out[got] = (float)strtod(p, &end);
    if (end == p) break;
    p = end;
  }
  return got;
}

Command parse_command(const char *line) {
  Command c;
  const char *p = skip(line);
  if (*p == '\0') return c;
  const char k = *p++;
  switch (k) {
    case 'J':
      c.type = read_floats(p, c.v, 3) == 3 ? CMD_JOINTS : CMD_ERROR;
      break;
    case 'P':
      c.type = read_floats(p, c.v, 3) == 3 ? CMD_POSE : CMD_ERROR;
      break;
    case 'H':
      c.type = CMD_HOME;
      break;
    case 'C':
      c.type = CMD_CALIBRATE;
      break;
    case '?':
      c.type = CMD_INFO;
      break;
    case 'M': {
      float m;
      if (read_floats(p, &m, 1) == 1 && m >= 0 && m <= 2) {
        c.type = CMD_MODE;
        c.mode = (int)m;
      } else {
        c.type = CMD_ERROR;
      }
      break;
    }
    default:
      c.type = CMD_ERROR;
  }
  return c;
}
