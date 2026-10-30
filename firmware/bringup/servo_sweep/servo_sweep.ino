// Bring-up step 1: are the servos alive and powered correctly?
// Moves each servo slowly 60..120 deg (SAFE range, links may be attached).
// Watch for: jitter (bad power/ground), brown-out resets (supply too weak).
#include <Servo.h>

static const uint8_t PINS[3] = {9, 10, 11};
Servo s[3];

void setup() {
  Serial.begin(115200);
  for (int i = 0; i < 3; ++i) {
    s[i].attach(PINS[i], 900, 2100);
    s[i].writeMicroseconds(1500);  // centre
  }
  Serial.println(F("servo_sweep: all servos at centre (1500 us). Sweeping in 2 s..."));
  delay(2000);
}

void loop() {
  for (int i = 0; i < 3; ++i) {
    Serial.print(F("servo "));
    Serial.println(i + 1);
    for (int us = 1500; us <= 1833; us += 4) { s[i].writeMicroseconds(us); delay(15); }   // +30 deg
    for (int us = 1833; us >= 1167; us -= 4) { s[i].writeMicroseconds(us); delay(15); }   // -30 deg
    for (int us = 1167; us <= 1500; us += 4) { s[i].writeMicroseconds(us); delay(15); }
    delay(500);
  }
}
