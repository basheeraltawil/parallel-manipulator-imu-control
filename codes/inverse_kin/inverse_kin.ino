#include <math.h>
#include <Wire.h>
#include <Servo.h>
int L=50; //links length
float px1,py1;  //x,y end effector cooordination
float r=26,fi=30;//platform radius and orientation
//gyroscope and servo
Servo tests;        //initialize a servo object for the connected servo  
const int MPU_ADDR = 0x68; // I2C address of the MPU-6050. If AD0 pin is set to HIGH, the I2C address will be 0x69.
int16_t accelerometer_x, accelerometer_y, accelerometer_z; // variables for accelerometer raw data
int16_t gyro_x, gyro_y, gyro_z; // variables for gyro raw data
int16_t temperature; // variables for temperature data
char tmp_str[7]; // temporary variable used in convert function
int16_t xs=0,ys=0,zs=0;
int16_t xgs=0,ygs=0,zgs=0;
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
}

void loop() {
Wire.beginTransmission(MPU_ADDR);
Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H) [MPU-6000 and MPU-6050 Register Map and Descriptions Revision 4.2, p.40]
Wire.endTransmission(false); // the parameter indicates that the Arduino will send a restart. As a result, the connection is kept active.
Wire.requestFrom(MPU_ADDR, 7*2, true); // request a total of 7*2=14 registers
// "Wire.read()<<8 | Wire.read();" means two registers are read and stored in the same variable
accelerometer_x = Wire.read()<<8 | Wire.read(); // reading registers: 0x3B (ACCEL_XOUT_H) and 0x3C (ACCEL_XOUT_L)
accelerometer_y = Wire.read()<<8 | Wire.read(); // reading registers: 0x3D (ACCEL_YOUT_H) and 0x3E (ACCEL_YOUT_L)
accelerometer_z = Wire.read()<<8 | Wire.read(); // reading registers: 0x3F (ACCEL_ZOUT_H) and 0x40 (ACCEL_ZOUT_L)
temperature = Wire.read()<<8 | Wire.read(); // reading registers: 0x41 (TEMP_OUT_H) and 0x42 (TEMP_OUT_L)
gyro_x = Wire.read()<<8 | Wire.read(); // reading registers: 0x43 (GYRO_XOUT_H) and 0x44 (GYRO_XOUT_L)
gyro_y = Wire.read()<<8 | Wire.read(); // reading registers: 0x45 (GYRO_YOUT_H) and 0x46 (GYRO_YOUT_L)
gyro_z = Wire.read()<<8 | Wire.read(); // reading registers: 0x47 (GYRO_ZOUT_H) and 0x48 (GYRO_ZOUT_L)
//accelerometer mapping values 
px1=map(accelerometer_x,-32768,32768,-20,20);
py1=map(accelerometer_y,-32768,32768,-20,20);
zs=map(accelerometer_y,-32768,32767,0,180);
//delay(1000);
//gyroscope mapping values
xgs=map(gyro_x,-500,1700,0,180);
ygs=map(gyro_y,-32768,32767,0,180);
zgs=map(gyro_z,-32768,32767,0,180);
//tests.write(ys);                 //command to rotate the servo to the specified angle
 
//robot geometrics and inverse kinematics
float rx=r*cos(30*PI/180),ry=r*sin(30*PI/180);//platform center
float a=sqrt((pow(px1,2)+pow(py1,2))/4),h=sqrt(pow(L,2)-pow(a,2));//geometrics
float alfa=atan2(h,a)*180/PI;
float beta=atan2(py1,px1)*180/PI;
float theta=alfa+beta;
tests.write(theta);
Serial.print("x cor\t  ");
Serial.println(px1);
Serial.print("y cor\t  ");
Serial.println(py1);
delay(500);
}
