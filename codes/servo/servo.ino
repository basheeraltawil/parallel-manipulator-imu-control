#include <Servo.h>             //Servo library

Servo servo_test;        //initialize a servo object for the connected servo  
                
int angle = 0;    
 
void setup() 
{ 
  servo_test.attach(9);      // attach the signal pin of servo to pin9 of arduino
 
/*for(angle = 0; angle <180; angle += 1)    // command to move from 0 degrees to 180 degrees 
  {                                  
    servo_test.write(angle);                 //command to rotate the servo to the specified angle
    delay(20);  
                         
  }
 for(angle =180; angle>=1; angle-=1)     // command to move from 180 degrees to 0 degrees 
  {                                
    servo_test.write(angle);              //command to rotate the servo to the specified angle
    delay(20);
                         
  } */
  
  
 
} 
  
void loop() 
{ 
  for(angle = 0; angle <150; angle += 1)    // command to move from 0 degrees to 180 degrees 
  {                                 
    servo_test.write(angle);                 //command to rotate the servo to the specified angle
    delay(20);                       
  } 
 
  delay(1000);
  
  /*for(angle = 60; angle>=1; angle-=1)     // command to move from 180 degrees to 0 degrees 
  {                                
    servo_test.write(angle);              //command to rotate the servo to the specified angle
    delay(20);                       
  } 

    delay(1000);*/
     
}
