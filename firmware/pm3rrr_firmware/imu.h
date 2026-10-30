// MPU6050 driver (raw I2C, no library needed) + complementary filter.
#pragma once
#include <stdint.h>

struct ImuSample {
  float ax, ay, az;  // m/s^2
  float gx, gy, gz;  // rad/s (bias removed after calibration)
};

class Mpu6050 {
 public:
  explicit Mpu6050(uint8_t addr) : addr_(addr) {}
  bool begin();                       // wake up, +-2 g, +-250 dps, 44 Hz DLPF
  bool read(ImuSample &s);            // false on I2C error
  void calibrate(uint16_t n = 200);   // gyro bias, sensor must be STILL
  bool present() const { return ok_; }

 private:
  uint8_t addr_;
  bool ok_ = false;
  float bias_[3] = {0, 0, 0};
  bool readRaw(int16_t v[7]);
};

class ComplementaryFilter {
 public:
  explicit ComplementaryFilter(float tau_s) : tau_(tau_s) {}
  void update(const ImuSample &s, float dt);
  float roll = 0, pitch = 0;  // rad

 private:
  float tau_;
  bool init_ = false;
};
