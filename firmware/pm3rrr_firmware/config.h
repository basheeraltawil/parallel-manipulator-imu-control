// =====================================================================
//  pm3rrr firmware - configuration (edit THIS file for your robot)
//  Geometry must match ros2_ws/src/pm3rrr_description/config/robot_params.yaml
// =====================================================================
#pragma once

// ---------------------------------------------------------------- pins
// Arduino Uno/Nano: any digital pin works with the Servo library.
static const uint8_t SERVO_PIN[3] = {9, 10, 11};
// MPU6050: SDA=A4, SCL=A5 (Uno/Nano). Hand-held IMU AD0=GND -> 0x68,
// optional platform/base IMU AD0=VCC -> 0x69.
static const uint8_t IMU_HAND_ADDR = 0x68;
static const uint8_t IMU_AUX_ADDR = 0x69;

// ------------------------------------------------------------ geometry
// (millimetres / degrees, measured from the CAD parts)
#define PM_BASE_RADIUS 87.5f
#define PM_PLATFORM_RADIUS 26.0f
#define PM_L1 50.0f
#define PM_L2 50.0f
static const float PM_BASE_ANGLE_DEG[3] = {-30.0f, 90.0f, 210.0f};
static const float PM_PLATFORM_ANGLE_DEG[3] = {30.0f, 150.0f, 270.0f};
static const float PM_WORKING_MODE[3] = {-1.0f, -1.0f, -1.0f};  // theta = beta - alpha

// ------------------------------------------------------------- servos
// World angle of each motor link when the servo is at its centre (90 deg).
static const float SERVO_CENTER_DEG[3] = {94.286907f, -145.713093f, -25.713093f};
static const float SERVO_DIRECTION[3] = {1.0f, 1.0f, 1.0f};
// Calibration trims [us], found with bringup/servo_calibration (horns are
// splined in ~7 deg steps, so every robot needs its own trim!)
static const int SERVO_TRIM_US[3] = {0, 0, 0};
#define SERVO_US_MIN 1000          // pulse at 0 deg   (check your servo!)
#define SERVO_US_MAX 2000          // pulse at 180 deg
#define SERVO_LIMIT_DEG 85.0f      // +- travel around the centre that we allow
#define SERVO_MAX_SPEED_DEG_S 300.0f  // slew limit applied in firmware

// ------------------------------------------------------------- timing
#define CONTROL_PERIOD_MS 20       // 50 Hz servo update
#define IMU_PERIOD_MS 10           // 100 Hz IMU + filter
#define STATUS_PERIOD_MS 100       // 10 Hz telemetry
#define WATCHDOG_MS 500            // stream mode: hold if the PC goes silent

// ------------------------------------------------------- teleoperation
#define TELE_MAX_TILT_DEG 25.0f
#define TELE_DEADBAND_DEG 3.0f
#define TELE_EXPO 0.35f
#define TELE_REACH_MM 16.0f
#define TELE_MAX_SPEED_MM_S 50.0f
#define CF_TAU_S 0.4f              // complementary filter time constant
// Singularity guard (cheap version of the PC's condition-number check, see
// docs/learning/06): together these two bounds keep kappa below ~25.
#define SINGULARITY_GUARD_ENABLED 1
#define TYPE2_MIN 0.25f            // min normalised det(Jx)  (parallel singularity, signed!)
#define TYPE1_MIN 0.30f            // min normalised |Jq_ii|  (serial singularity: leg stretched)
