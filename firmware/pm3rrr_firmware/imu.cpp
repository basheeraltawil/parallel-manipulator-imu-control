#include "imu.h"

#include <Arduino.h>
#include <Wire.h>
#include <math.h>

static const float G = 9.80665f;

static void writeReg(uint8_t addr, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission(true);
}

bool Mpu6050::begin() {
  Wire.beginTransmission(addr_);
  ok_ = (Wire.endTransmission(true) == 0);
  if (!ok_) return false;
  writeReg(addr_, 0x6B, 0x01);  // PWR_MGMT_1: wake up, PLL with X gyro (stable clock)
  writeReg(addr_, 0x1A, 0x03);  // CONFIG: DLPF 44 Hz (cuts servo vibration)
  writeReg(addr_, 0x1B, 0x00);  // GYRO_CONFIG: +-250 dps -> 131 LSB/dps
  writeReg(addr_, 0x1C, 0x00);  // ACCEL_CONFIG: +-2 g -> 16384 LSB/g
  return true;
}

bool Mpu6050::readRaw(int16_t v[7]) {
  Wire.beginTransmission(addr_);
  Wire.write(0x3B);  // ACCEL_XOUT_H
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(addr_, (uint8_t)14, (uint8_t)true) != 14) return false;
  for (int i = 0; i < 7; ++i) v[i] = (int16_t)((Wire.read() << 8) | Wire.read());
  return true;
}

bool Mpu6050::read(ImuSample &s) {
  int16_t v[7];
  if (!readRaw(v)) return false;
  s.ax = v[0] / 16384.0f * G;
  s.ay = v[1] / 16384.0f * G;
  s.az = v[2] / 16384.0f * G;
  // v[3] = temperature
  const float k = (PI / 180.0f) / 131.0f;
  s.gx = v[4] * k - bias_[0];
  s.gy = v[5] * k - bias_[1];
  s.gz = v[6] * k - bias_[2];
  return true;
}

void Mpu6050::calibrate(uint16_t n) {
  float sum[3] = {0, 0, 0};
  bias_[0] = bias_[1] = bias_[2] = 0;
  ImuSample s;
  uint16_t got = 0;
  for (uint16_t i = 0; i < n; ++i) {
    if (read(s)) {
      sum[0] += s.gx;
      sum[1] += s.gy;
      sum[2] += s.gz;
      ++got;
    }
    delay(5);
  }
  if (got) {
    for (int i = 0; i < 3; ++i) bias_[i] = sum[i] / got;
  }
}

void ComplementaryFilter::update(const ImuSample &s, float dt) {
  const float r_acc = atan2f(s.ay, s.az);
  const float p_acc = atan2f(-s.ax, sqrtf(s.ay * s.ay + s.az * s.az));
  if (!init_) {
    roll = r_acc;
    pitch = p_acc;
    init_ = true;
    return;
  }
  const float a = tau_ / (tau_ + dt);
  roll = a * (roll + s.gx * dt) + (1.0f - a) * r_acc;
  pitch = a * (pitch + s.gy * dt) + (1.0f - a) * p_acc;
}
