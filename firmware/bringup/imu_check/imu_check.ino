// Bring-up step 2: does the MPU6050 answer, and are the axes what we think?
// Prints accel [m/s^2], gyro [deg/s] and the filtered roll/pitch [deg].
// Expected when flat and still: az ~ +9.8, gyro ~ 0 (+- bias), roll/pitch ~ 0.
// Tilt "nose down" -> pitch positive -> the platform will move +x.
#include <Wire.h>

static const uint8_t ADDR = 0x68;  // 0x69 if AD0 is tied high

static void wr(uint8_t reg, uint8_t v) {
  Wire.beginTransmission(ADDR); Wire.write(reg); Wire.write(v); Wire.endTransmission(true);
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.beginTransmission(ADDR);
  if (Wire.endTransmission(true) != 0) {
    Serial.println(F("MPU6050 NOT FOUND - check VCC/GND/SDA(A4)/SCL(A5) and AD0"));
    while (true) {}
  }
  wr(0x6B, 0x01); wr(0x1A, 0x03); wr(0x1B, 0x00); wr(0x1C, 0x00);
  Serial.println(F("ax ay az [m/s2] | gx gy gz [deg/s] | roll pitch [deg]"));
}

float roll = 0, pitch = 0;
unsigned long last = 0;

void loop() {
  Wire.beginTransmission(ADDR); Wire.write(0x3B); Wire.endTransmission(false);
  Wire.requestFrom(ADDR, (uint8_t)14, (uint8_t)true);
  int16_t v[7];
  for (int i = 0; i < 7; ++i) v[i] = (Wire.read() << 8) | Wire.read();
  const float ax = v[0] / 16384.0 * 9.81, ay = v[1] / 16384.0 * 9.81, az = v[2] / 16384.0 * 9.81;
  const float gx = v[4] / 131.0, gy = v[5] / 131.0, gz = v[6] / 131.0;
  const unsigned long now = millis();
  const float dt = last ? (now - last) * 1e-3 : 0.01;
  last = now;
  const float a = 0.4 / (0.4 + dt);  // complementary filter, tau = 0.4 s
  roll = a * (roll + gx * dt) + (1 - a) * degrees(atan2(ay, az));
  pitch = a * (pitch + gy * dt) + (1 - a) * degrees(atan2(-ax, sqrt(ay * ay + az * az)));
  char buf[96];
  snprintf(buf, sizeof(buf), "%6d %6d %6d | %6d %6d %6d | %5d %5d (x100)",
           (int)(ax * 100), (int)(ay * 100), (int)(az * 100), (int)(gx * 100), (int)(gy * 100),
           (int)(gz * 100), (int)(roll * 100), (int)(pitch * 100));
  Serial.println(buf);
  delay(20);
}
