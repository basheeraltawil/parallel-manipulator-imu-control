#include <math.h>
#include <Wire.h>
#include <Servo.h>
int del=100;
int L = 50; //links length
//float pxi1, pyi1,pxi2,pyi2,pxi3,pyi3; //x,y end effector cooordination
float pxi1=52, pyi1=53,pxi2=-72,pyi2=18,pxi3=22.5,pyi3=-72;
float pxf1, pyf1,pxf2,pyf2,pxf3,pyf3; //x,y end effector to be reached
float dx=10;  //x-displacement 
float dy=0;   //y-displacement

Servo tests,tests2,tests3;        //initialize a servo object for the connected servo
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  tests.attach(9);
  tests2.attach(10);
  tests3.attach(11);
   

}

void loop() {
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
    Serial.println(theta);
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
    Serial.println(theta);
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
    Serial.println(theta);
      delay(del);
    }
     
    }
  }

  //for second manipulator
  
  float a2 = sqrt((pow(pxf2, 2) + pow(pyf2, 2)) / 4); //geometrics
  float h2 = sqrt(pow(L, 2) - pow(a2, 2));
  float alfa2 = atan2(h2, a2) * 180 / PI;
  float beta2 = atan2(pyf2, pxf2) * 180 / PI;

  float  betaf2, thet2,theta2;
  if (beta2 < 0)
  {
    betaf2=360+beta2;
    theta2 = betaf2+alfa2;
     for(int thetar2=0;thetar2<=theta2;thetar2++)
     {
      tests2.write(thetar2);
     Serial.print("servo2\t  ");
     
    Serial.println(thetar2);
    delay(del);
     }
    
  }
  else
  {
    thet2 =alfa2 + abs(beta2);
    if (thet2 < 0)
    {
       for(int thetar2=0;thetar2<=theta2;thetar2++)
     {
      tests2.write(thetar2);
     Serial.print("servo2\t  ");
     
    Serial.println(thetar2);
    delay(del);
     }
    }
    else
    {
      theta2 = thet2;
       for(int thetar2=0;thetar2<=theta2;thetar2++)
     {
      tests2.write(thetar2);
     Serial.print("servo2\t  ");
     
    Serial.println(thetar2);
    delay(del);
     }
    }
  }

 //for third manipulator

  float a3 = sqrt((pow(pxf3, 2) + pow(pyf3, 2)) / 4); //geometrics
  float h3 = sqrt(pow(L, 2) - pow(a3, 2));
  float alfa3 = atan2(h3, a3) * 180 / PI;
  float beta3 = atan2(pyf3, pxf3) * 180 / PI;

  float betaf3, thet3,theta3;
  if (beta3 < 0)
  {
    betaf3 = 360+beta3;
    theta3 =  betaf3+alfa3;
      for(int thetar3=0;thetar3<=theta3;thetar3++)
     {
      tests3.write(thetar3);
     Serial.print("servo2\t  ");
     
    Serial.println(thetar3);
    delay(del);
     }
  }
  else
  {
    thet3 =alfa3 + abs(beta3);
    if (thet3 < 0)
    {
      theta3 = thet3 + 360;
       for(int thetar3=0;thetar3<=theta3;thetar3++)
     {
      tests3.write(thetar3);
     Serial.print("servo2\t  ");
     
    Serial.println(thetar3);
    delay(del);
     }
    }
    else
    {
      theta3 = thet3;
       for(int thetar3=0;thetar3<=theta3;thetar3++)
     {
      tests3.write(thetar3);
     Serial.print("servo2\t  ");
     
    Serial.println(thetar3);
    delay(del);
     }
    }
  }
  
}
