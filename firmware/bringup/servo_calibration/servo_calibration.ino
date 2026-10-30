// Bring-up step 3: find each servo's trim so that "90 deg" really is the
// mounting angle used by the kinematics (SERVO_CENTER_DEG in config.h).
//
// Procedure (docs/industrialization/05_calibration_and_commissioning.md):
//  1. Mount the proximal links, NOT the distal links.
//  2. Put the printed alignment jig (or a protractor) over servo i.
//  3. Send  "<i> <us>"  e.g. "1 1512"  until the link points exactly along
//     its home direction (theta_home = SERVO_CENTER_DEG[i]).
//  4. Copy  trim = us - 1500  into SERVO_TRIM_US in config.h.
#include <Servo.h>

static const uint8_t PINS[3] = {9, 10, 11};
Servo s[3];
int us[3] = {1500, 1500, 1500};

void setup() {
  Serial.begin(115200);
  for (int i = 0; i < 3; ++i) { s[i].attach(PINS[i], 900, 2100); s[i].writeMicroseconds(us[i]); }
  Serial.println(F("servo_calibration: send '<servo 1-3> <pulse us>', e.g. '2 1490'"));
}

void loop() {
  if (Serial.available()) {
    const int i = Serial.parseInt();
    const int p = Serial.parseInt();
    if (i >= 1 && i <= 3 && p >= 1000 && p <= 2000) {
      us[i - 1] = p;
      s[i - 1].writeMicroseconds(p);
      Serial.print(F("servo ")); Serial.print(i); Serial.print(F(" -> ")); Serial.print(p);
      Serial.print(F(" us   trim = ")); Serial.println(p - 1500);
    }
    while (Serial.available()) Serial.read();
  }
}
