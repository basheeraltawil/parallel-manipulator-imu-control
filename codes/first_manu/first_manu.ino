#include <math.h>
#include <Wire.h>
#include <Servo.h>
int del=10;
int L = 50; //links length
//float pxi1, pyi1,pxi2,pyi2,pxi3,pyi3; //x,y end effector cooordination
float pxi1=52, pyi1=53,pxi2=-72,pyi2=18,pxi3=22.5,pyi3=-72;
float pxf1, pyf1,pxf2,pyf2,pxf3,pyf3; //x,y end effector to be reached
float dx=-20;  //x-displacement 
float dy=0;   //y-displacement

Servo tests,tests2,tests3;        //initialize a servo object for the connected servo
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  tests.attach(9);
  pxf1=pxi1+dx, pyf1=pyi1+dy,pxf2=pxi2+dx,pyf2=pyi2+dy,pxf3=pxi3+dx,pyf3=pyi3+dy;
  //robot geometrics and inverse kinematics
  //for first manipulator
  float a = sqrt((pow(pxf1, 2) + pow(pyf1, 2)) / 4); //geometrics
  float h = sqrt(pow(L, 2) - pow(a, 2));
  float alfa = atan2(h, a) * 180 / PI;
  float beta = atan2(pyf1, pxf1) * 180 / PI;

  float theta, betaf, thet1;
  if (beta < 0)
  {
    betaf=360+beta;
    theta = betaf+alfa;
    for(int thetar=0;thetar<=theta;thetar++)
    {
      tests.write(thetar);
      Serial.print("servo1\t  ");
    Serial.println(thetar);
    delay(del);
    }
    
  }
  else
  {
    thet1 =alfa + abs(beta);
    if (thet1 < 0)
    {
      theta = thet1 + 360;
       for(int thetar=0;thetar<=theta;thetar++)
    {
      tests.write(thetar);
        Serial.print("servo1\t  ");
    Serial.println(thetar);
      delay(del);
    }
    
    }
    else
    {
      theta = thet1;
       for(int thetar=0;thetar<=theta;thetar++)
    {
      tests.write(thetar);
       Serial.print("servo1\t  ");
    Serial.println(thetar);
      delay(del);
    }
     
    }
  }
 
}

void loop() {
   

}
