// Host-side unit test of the firmware's pure C++ modules (no Arduino needed).
//   make -C firmware/test
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <sstream>
#include <string>

#include "../pm3rrr_firmware/kinematics.h"
#include "../pm3rrr_firmware/protocol.h"

static int failures = 0;
#define CHECK(cond, msg)                         \
  do {                                           \
    if (!(cond)) {                               \
      std::printf("FAIL: %s\n", msg);            \
      ++failures;                                \
    }                                            \
  } while (0)

int main(int argc, char **argv) {
  pm_kinematics_init();
  // 1) home pose -> all servos at 90 deg
  IkResult h = pm_ik({0, 0, 0});
  CHECK(h.ok(), "home must be valid");
  for (int i = 0; i < 3; ++i) CHECK(std::fabs(h.servo_deg[i] - 90.0f) < 0.01f, "home servo != 90 deg");

  // 2) compare with the Python reference implementation
  std::ifstream f(argc > 1 ? argv[1] : "expected_ik.csv");
  std::string line;
  std::getline(f, line);
  int n = 0, okMismatch = 0;
  float worst = 0;
  while (std::getline(f, line)) {
    float v[7];
    std::stringstream ss(line);
    for (int i = 0; i < 7; ++i) {
      std::string tok;
      std::getline(ss, tok, ',');
      v[i] = std::stof(tok);
    }
    IkResult r = pm_ik({v[0], v[1], v[2] * 3.14159265358979f / 180.0f});
    const bool expect_ok = v[6] > 0.5f;
    if (r.ok() != expect_ok) ++okMismatch;
    if (r.reachable)
      for (int i = 0; i < 3; ++i) worst = std::fmax(worst, std::fabs(r.servo_deg[i] - v[3 + i]));
    ++n;
  }
  CHECK(n > 100, "reference file missing or too short");
  CHECK(worst < 0.01f, "servo angles differ from Python by > 0.01 deg");
  CHECK(okMismatch <= n / 100, "validity flags differ from Python on > 1 % of poses (float rounding at borders)");
  std::printf("IK: %d poses, worst servo difference %.5f deg, validity mismatches %d\n", n, worst, okMismatch);

  // 3) protocol parser
  Command c = parse_command("J 90.5 80 100.25");
  CHECK(c.type == CMD_JOINTS && std::fabs(c.v[2] - 100.25f) < 1e-5f, "parse J");
  CHECK(parse_command("P 1 2").type == CMD_ERROR, "P needs 3 numbers");
  CHECK(parse_command("M 1").type == CMD_MODE && parse_command("M 1").mode == 1, "parse M");
  CHECK(parse_command("M 7").type == CMD_ERROR, "mode range");
  CHECK(parse_command("  H").type == CMD_HOME, "parse H with spaces");
  CHECK(parse_command("X").type == CMD_ERROR, "unknown command");
  CHECK(parse_command("").type == CMD_NONE, "empty line");

  std::printf(failures ? "FAILED (%d)\n" : "ALL FIRMWARE HOST TESTS PASSED\n", failures);
  return failures ? 1 : 0;
}
