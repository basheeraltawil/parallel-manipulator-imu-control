// =====================================================================
//  pm3rrr firmware - planar 3-RRR parallel manipulator with IMU control
//  Board: Arduino Uno / Nano (ATmega328P) or any Arduino-compatible board
//
//  Replaces the original experimental sketches (legacy/codes). Main fixes:
//   * one clean inverse kinematics for all three legs, with reachability,
//     servo-limit and singularity checks  (kinematics.cpp)
//   * servo commands with writeMicroseconds() (0.18 deg) instead of
//     write(int deg) (1 deg), plus a slew-rate limit
//   * MPU6050 gyro-bias calibration + complementary filter: tilt ANGLE,
//     not raw gyro RATE, drives the platform  (imu.cpp)
//   * non-blocking scheduler (no delay() in the loop), watchdog, modes
//
//  Modes:  0 = STREAM (PC/ROS 2 sends J or P lines)
//          1 = IMU    (stand-alone: hand-held MPU6050 tilt -> platform)
//          2 = HOLD   (servos hold the last pose)
// =====================================================================
#include <Servo.h>
#include <Wire.h>

#include "config.h"
#include "imu.h"
#include "kinematics.h"
#include "protocol.h"

enum Mode { MODE_STREAM = 0, MODE_IMU = 1, MODE_HOLD = 2 };

static Servo servo[3];
static Mpu6050 imuHand(IMU_HAND_ADDR);
static ComplementaryFilter cf(CF_TAU_S);
static Mode mode = MODE_HOLD;

static float servoTarget[3] = {90, 90, 90};   // deg, what we want
static float servoNow[3] = {90, 90, 90};      // deg, slew-limited output
static Pose teleTarget = {0, 0, 0};
static unsigned long tControl = 0, tImu = 0, tStatus = 0, tLastCmd = 0;
static char lineBuf[64];
static uint8_t lineLen = 0;

// ------------------------------------------------------------ helpers
static void writeServos() {
  for (int i = 0; i < 3; ++i) {
    const int us = SERVO_US_MIN + (int)(servoNow[i] / 180.0f * (SERVO_US_MAX - SERVO_US_MIN) + 0.5f) +
                   SERVO_TRIM_US[i];
    servo[i].writeMicroseconds(us);
  }
}

static bool setPose(const Pose &p) {
  IkResult r = pm_ik(p);
  if (!r.ok()) {
    Serial.print(F("E rejected pose: "));
    Serial.println(!r.reachable ? F("unreachable") : !r.within_limits ? F("servo limit") : F("near singularity"));
    return false;
  }
  for (int i = 0; i < 3; ++i) servoTarget[i] = r.servo_deg[i];
  return true;
}

static bool setJoints(const float deg[3]) {
  for (int i = 0; i < 3; ++i) {
    if (deg[i] < 90.0f - SERVO_LIMIT_DEG || deg[i] > 90.0f + SERVO_LIMIT_DEG) {
      Serial.println(F("E rejected joints: servo limit"));
      return false;
    }
  }
  for (int i = 0; i < 3; ++i) servoTarget[i] = deg[i];
  return true;
}

static void goHome() {
  Pose home = {0, 0, 0};
  teleTarget = home;
  setPose(home);
}

static void calibrateImu() {
  if (!imuHand.present()) {
    Serial.println(F("E no hand IMU"));
    return;
  }
  Serial.println(F("OK calibrating gyro - keep the IMU still"));
  imuHand.calibrate(200);
  Serial.println(F("OK calibration done"));
}

// teleoperation shaping: dead band -> normalise -> expo
static float shape(float angle) {
  const float db = TELE_DEADBAND_DEG * PI / 180.0f, mx = TELE_MAX_TILT_DEG * PI / 180.0f;
  float a = fabsf(angle) - db;
  if (a < 0) a = 0;
  float n = a / (mx - db);
  if (n > 1) n = 1;
  n = (1 - TELE_EXPO) * n + TELE_EXPO * n * n * n;
  return angle < 0 ? -n : n;
}

static void teleopStep(float dt) {
  float gx = TELE_REACH_MM * shape(cf.pitch);
  float gy = TELE_REACH_MM * shape(cf.roll);
  float dx = gx - teleTarget.x, dy = gy - teleTarget.y;
  const float d = sqrtf(dx * dx + dy * dy), lim = TELE_MAX_SPEED_MM_S * dt;
  if (d > lim) {
    dx *= lim / d;
    dy *= lim / d;
  }
  Pose cand = {teleTarget.x + dx, teleTarget.y + dy, 0};
  IkResult r = pm_ik(cand);
  if (r.ok()) {  // never step into an unsafe pose: just stay
    teleTarget = cand;
    for (int i = 0; i < 3; ++i) servoTarget[i] = r.servo_deg[i];
  }
}

static void handleLine(const char *line) {
  Command c = parse_command(line);
  switch (c.type) {
    case CMD_JOINTS:
      if (mode == MODE_STREAM && setJoints(c.v)) tLastCmd = millis();
      break;
    case CMD_POSE: {
      Pose p = {c.v[0], c.v[1], c.v[2] * PI / 180.0f};
      if (mode == MODE_STREAM && setPose(p)) tLastCmd = millis();
      break;
    }
    case CMD_HOME:
      goHome();
      break;
    case CMD_MODE:
      mode = (Mode)c.mode;
      tLastCmd = millis();
      Serial.print(F("OK mode "));
      Serial.println(c.mode);
      break;
    case CMD_CALIBRATE:
      calibrateImu();
      break;
    case CMD_INFO:
      Serial.println(F("OK pm3rrr_firmware 1.0 (3-RRR, MG996R, MPU6050)"));
      break;
    case CMD_ERROR:
      Serial.print(F("E bad command: "));
      Serial.println(line);
      break;
    default:
      break;
  }
}

static void readSerial() {
  while (Serial.available()) {
    const char ch = (char)Serial.read();
    if (ch == '\n' || ch == '\r') {
      if (lineLen) {
        lineBuf[lineLen] = '\0';
        handleLine(lineBuf);
        lineLen = 0;
      }
    } else if (lineLen < sizeof(lineBuf) - 1) {
      lineBuf[lineLen++] = ch;
    } else {
      lineLen = 0;  // overflow: drop the line
    }
  }
}

// --------------------------------------------------------------- main
void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);
  pm_kinematics_init();
  for (int i = 0; i < 3; ++i) servo[i].attach(SERVO_PIN[i], SERVO_US_MIN - 100, SERVO_US_MAX + 100);
  goHome();
  for (int i = 0; i < 3; ++i) servoNow[i] = servoTarget[i];
  writeServos();
  if (imuHand.begin()) {
    delay(500);  // let the robot settle before calibrating
    calibrateImu();
    mode = MODE_IMU;
  } else {
    Serial.println(F("E hand IMU not found at 0x68 - STREAM mode"));
    mode = MODE_STREAM;
  }
  tControl = tImu = tStatus = tLastCmd = millis();
  Serial.println(F("OK ready"));
}

void loop() {
  readSerial();
  const unsigned long now = millis();

  if (now - tImu >= IMU_PERIOD_MS) {
    const float dt = (now - tImu) * 1e-3f;
    tImu = now;
    ImuSample s;
    if (imuHand.present() && imuHand.read(s)) {
      cf.update(s, dt);
      if (mode == MODE_IMU) teleopStep(dt);
      Serial.print('I');
      const float v[6] = {s.ax, s.ay, s.az, s.gx, s.gy, s.gz};
      for (int i = 0; i < 6; ++i) {
        Serial.print(' ');
        Serial.print(v[i], 4);
      }
      Serial.println();
    }
  }

  if (now - tControl >= CONTROL_PERIOD_MS) {
    const float dt = (now - tControl) * 1e-3f;
    tControl = now;
    if (mode == MODE_STREAM && now - tLastCmd > WATCHDOG_MS) {
      // PC silent: hold the current output (do not jump anywhere)
      for (int i = 0; i < 3; ++i) servoTarget[i] = servoNow[i];
    }
    const float step = SERVO_MAX_SPEED_DEG_S * dt;
    for (int i = 0; i < 3; ++i) {
      float e = servoTarget[i] - servoNow[i];
      if (e > step) e = step;
      if (e < -step) e = -step;
      servoNow[i] += e;
    }
    writeServos();
  }

  if (now - tStatus >= STATUS_PERIOD_MS) {
    tStatus = now;
    Serial.print('S');
    for (int i = 0; i < 3; ++i) {
      Serial.print(' ');
      Serial.print(servoNow[i], 2);
    }
    Serial.print(' ');
    Serial.println((int)mode);
  }
}
