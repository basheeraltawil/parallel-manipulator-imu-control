#include <math.h>
#include <Wire.h>
#include <Servo.h>
int del=10;
float xi = -10, xf = 12, yi =-8, yf = 15;
int L = 50; //links length
float xp = 76; //footprint x extention
float yp = 44; //footprint y extention
float yR = 87.5; //radius of second manipulator
float px1, py1,px2,py2,px3,py3; //x,y end effector cooordination
float r = 26, fi = 30; //platform radius and orientation
float dx;   //platform x-center cordination gyro
float dy; //platform y-center cordination gyro
float rx = r * cos(30 * PI / 180), ry = r * sin(30 * PI / 180); //platform center
//gyroscope and servo
Servo tests,tests2,tests3;        //initialize a servo object for the connected servo
const int MPU_ADDR = 0x68; // I2C address of the MPU-6050. If AD0 pin is set to HIGH, the I2C address will be 0x69.
int16_t accelerometer_x, accelerometer_y, accelerometer_z; // variables for accelerometer raw data
int16_t gyro_x, gyro_y, gyro_z; // variables for gyro raw data
int16_t temperature; // variables for temperature data
char tmp_str[7]; // temporary variable used in convert function
int16_t xs = 0, ys = 0, zs = 0;
int16_t xgs = 0, ygs = 0, zgs = 0;
char* convert_int16_to_str(int16_t i)
{ // converts int16 to string. Moreover, resulting strings will have the same length in the debug monitor.
  sprintf(tmp_str, "%6d", i);
  return tmp_str;
}
//gyro and servo finish

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Wire.begin();
  Wire.beginTransmission(MPU_ADDR); // Begins a transmission to the I2C slave (GY-521 board)
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0); // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  tests.attach(9);
  tests2.attach(10);
  tests3.attach(11);
}

void loop() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H) [MPU-6000 and MPU-6050 Register Map and Descriptions Revision 4.2, p.40]
  Wire.endTransmission(false); // the parameter indicates that the Arduino will send a restart. As a result, the connection is kept active.
  Wire.requestFrom(MPU_ADDR, 7 * 2, true); // request a total of 7*2=14 registers
  // "Wire.read()<<8 | Wire.read();" means two registers are read and stored in the same variable
  accelerometer_x = Wire.read() << 8 | Wire.read(); // reading registers: 0x3B (ACCEL_XOUT_H) and 0x3C (ACCEL_XOUT_L)
  accelerometer_y = Wire.read() << 8 | Wire.read(); // reading registers: 0x3D (ACCEL_YOUT_H) and 0x3E (ACCEL_YOUT_L)
  accelerometer_z = Wire.read() << 8 | Wire.read(); // reading registers: 0x3F (ACCEL_ZOUT_H) and 0x40 (ACCEL_ZOUT_L)
  temperature = Wire.read() << 8 | Wire.read(); // reading registers: 0x41 (TEMP_OUT_H) and 0x42 (TEMP_OUT_L)
  gyro_x = Wire.read() << 8 | Wire.read(); // reading registers: 0x43 (GYRO_XOUT_H) and 0x44 (GYRO_XOUT_L)
  gyro_y = Wire.read() << 8 | Wire.read(); // reading registers: 0x45 (GYRO_YOUT_H) and 0x46 (GYRO_YOUT_L)
  gyro_z = Wire.read() << 8 | Wire.read(); // reading registers: 0x47 (GYRO_ZOUT_H) and 0x48 (GYRO_ZOUT_L)
  //accelerometer mapping values
  /*dx = map(accelerometer_x, -32768, 32767, xi, xf);
  dy = map(accelerometer_y, -32768, 32767, yi, yf);
  zs = map(accelerometer_y, -32768, 32767, 0, 180);*/
  //delay(1000);
  //gyroscope mapping values
  dx = map(gyro_x, -32768, 32768, xi, xf);
  dy= map(gyro_y, -32768, 32768, yi, yf);
  zgs = map(gyro_z, -32768, 32767, 0, 180);
  //tests.write(ys);                 //command to rotate the servo to the specified angle

  //robot geometrics and inverse kinematics
  //for first manipulator
  px1 = -(xp - 13) + dx; //cordination of x position of end effictor of manipulator one
  py1 = (yp + 7.5) + dy;  //cordination of y position of end effictor of manipulator one
  
  float a = sqrt((pow(px1, 2) + pow(py1, 2)) / 4); //geometrics
  float h = sqrt(pow(L, 2) - pow(a, 2));
  float alfa = atan2(h, a) * 180 / PI;
  float beta = atan2(py1, px1) * 180 / PI;

  float theta, theta1, thet1;
  if (beta < 0)
  {
    theta1 = (alfa + abs(beta));
    theta = 360 - theta1;
    tests.write(theta);
    Serial.print("servo1\t  ");
    Serial.println(theta);
    delay(del);
  }
  else
  {
    thet1 = -alfa + abs(beta);
    tests.write(theta);
    Serial.print("servo1\t  ");
    Serial.println(theta);
    delay(del);
    if (thet1 < 0)
    {
      theta = thet1 + 360;
      tests.write(theta);
      Serial.print("servo1\t  ");
    Serial.println(theta);
      delay(del);
    }
    else
    {
      theta = thet1;
      tests.write(theta);
      Serial.print("servo1\t  ");
    Serial.println(theta);
      delay(del);
    }
  }

  //for second manipulator
  px2 = dx-13; //cordination of x position of end effictor of manipulator one
  py2 = -(yR-7.5)+dy;  //cordination of y position of end effictor of manipulator one

  float a2 = sqrt((pow(px2, 2) + pow(py2, 2)) / 4); //geometrics
  float h2 = sqrt(pow(L, 2) - pow(a2, 2));
  float alfa2 = atan2(h2, a2) * 180 / PI;
  float beta2 = atan2(py2, px2) * 180 / PI;

  float theta22, theta2, thet2;
  if (beta2 < 0)
  {
    theta22 = (alfa2 + abs(beta2));
    theta2 = 360 - theta22;
    tests2.write(theta2);
     Serial.print("servo2\t  ");
    Serial.println(theta2);
    delay(del);
  }
  else
  {
    thet2 = -alfa2 + abs(beta2);
    tests2.write(theta2);
     Serial.print("servo2\t  ");
    Serial.println(theta2);
    delay(del);
    if (thet2 < 0)
    {
      theta2 = thet2 + 360;
      tests2.write(theta2);
      Serial.print("servo2\t  ");
    Serial.println(theta2);
      delay(del);
    }
    else
    {
      theta2 = thet2;
      tests2.write(theta2);
       Serial.print("servo2\t  ");
    Serial.println(theta2);
      delay(del);
    }
  }

 //for third manipulator
  px3 =xp+dx; //cordination of x position of end effictor of manipulator one
  py3 =(yp-15)+dy;  //cordination of y position of end effictor of manipulator one

  float a3 = sqrt((pow(px3, 2) + pow(py3, 2)) / 4); //geometrics
  float h3 = sqrt(pow(L, 2) - pow(a3, 2));
  float alfa3 = atan2(h3, a3) * 180 / PI;
  float beta3 = atan2(py3, px3) * 180 / PI;

  float theta33, theta3, thet3;
  if (beta3 < 0)
  {
    theta33 = (alfa3 + abs(beta3));
    theta3 = 360 - theta33;
    tests3.write(theta3);
     Serial.print("servo3\t  ");
    Serial.println(theta3);
    delay(del);
  }
  else
  {
    thet3 = -alfa3 + abs(beta3);
    tests3.write(theta3);
    Serial.print("servo3\t  ");
    Serial.println(theta3);
    delay(del);
    if (thet3 < 0)
    {
      theta3 = thet3 + 360;
      tests3.write(theta3);
      Serial.print("servo3\t  ");
    Serial.println(theta3);
      delay(del);
    }
    else
    {
      theta3 = thet3;
      tests3.write(theta3);
      Serial.print("servo3 \t");
    Serial.println(theta3);
      delay(del);
    }
  }

  
}
