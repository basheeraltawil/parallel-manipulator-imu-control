//####################INCLUDE LIBRARIES##################3
#include <math.h>
#include <Wire.h>
#include <Servo.h>
//servo

//###################DEFINITIONS AND CONSTANTS###################
//home position for servo
# define fs_home_pos 50    //first servo home position
# define sd_home_pos 50    //second servo home position
# define td_home_pos 50    //third servo home position
//minimum position for servo
# define fs_min_pos 0    //first servo minimum position
# define sd_min_pos 0    //second servo minimum position
# define td_min_pos 0    //third servo minimum position
//maximum position for servo
# define fs_max_pos 170    //first servo maximum position
# define sd_max_pos 170    //second servo maximum position
# define td_max_pos 170    //third servo maximum position
# define L 50  //Link lenght of arm
//#######################servo delays #########
#  define del 150
#  define sdelay 1000
//srrvo motor defintions
Servo tests1;
Servo tests2;
Servo tests3;
//###############GLOBAL VARIABLES####################3
//FIRST MANIPULATOR
float xi1;  //first manipulator intial x  end effector position
float yi1;  //first manipulator intial y end effector position
float xf1;  //first manipulator final x  end effector position
float yf1;  //first manipulator final y end effector position
float xd1;  //first manipulator x footprint position
float yd1;  //first manipulator y footprint position
//inverse kinematics variables-first manipulator
float a1;
float h1;
float alfa1;
float beta1;
float theta1;
//SECOND MANIPULATOR
float xi2;  //second manipulator intial x  end effector position
float yi2;  //second manipulator intial y end effector position
float xf2;  //second manipulator final x  end effector position
float yf2;  //second manipulator final y end effector position
float xd2;  //second manipulator x footprint position
float yd2;  //second manipulator y footprint position
//inverse kinematics variables-second manipulator
float a2;
float h2;
float alfa2;
float beta2;
float theta2;
//THIRD MANIPULATOR
float xi3;  //third manipulator intial x  end effector position
float yi3;  //third manipulator intial y end effector position
float xf3;  //third manipulator final x  end effector position
float yf3;  //third manipulator final y end effector position
float xd3;  //third manipulator x footprint position
float yd3;  //third manipulator y footprint position
//inverse kinematics variables-third manipulator
float a3;
float h3;
float alfa3;
float beta3;
float theta3;

void setup() {
  // put your setup code here, to run once:
Serial.begin(9600);
tests1.attach(9);
tests2.attach(10);
tests3.attach(11);
tests1.write(fs_home_pos); // move 1st servo to home postion
tests2.write(sd_home_pos); // move 2d servo to home postion
tests3.write(td_home_pos); // move 3rd  x   servo to home postion
}

void loop()
{
  // put your main code here, to run repeatedly:

}

       //##########################calculations and action functions#######
void first_manio_inv()
{
  //for first manipulator
  px1 = -(xp - 13) + dx; //cordination of x position of end effictor of manipulator one
  py1 = (yp + 7.5) + dy;  //cordination of y position of end effictor of manipulator one

  a1 = sqrt((pow(px1, 2) + pow(py1, 2)) / 4); //geometrics
  h1 = sqrt(pow(L, 2) - pow(a, 2));
  alfa1 = atan2(h, a) * 180 / PI;
  beta1 = atan2(py1, px1) * 180 / PI;

}
void second_manio_inv()
{
  //for second manipulator
  px2 = dx - 13; //cordination of x position of end effictor of manipulator one
  py2 = -(yR - 7.5) + dy; //cordination of y position of end effictor of manipulator one

  a2 = sqrt((pow(px2, 2) + pow(py2, 2)) / 4); //geometrics
  h2 = sqrt(pow(L, 2) - pow(a2, 2));
  alfa2 = atan2(h2, a2) * 180 / PI;
  beta2 = atan2(py2, px2) * 180 / PI;

}
void third_manio_inv()
{
  px3 = xp + dx; //cordination of x position of end effictor of manipulator one
  py3 = (yp - 15) + dy; //cordination of y position of end effictor of manipulator one

  a3 = sqrt((pow(px3, 2) + pow(py3, 2)) / 4); //geometrics
  h3 = sqrt(pow(L, 2) - pow(a3, 2));
  alfa3 = atan2(h3, a3) * 180 / PI;
  beta3 = atan2(py3, px3) * 180 / PI;  
}

void callman()
{
  first_manio_inv();
  third_manio_inv()
}
