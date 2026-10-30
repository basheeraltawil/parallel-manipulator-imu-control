#include <Servo.h>
Servo myservo;
int pos = 40;
int dela=100;


void setup()

{
  int deg =0;
int deg1 = 0;

  Serial.begin(9600);
   myservo.attach(9);
  myservo.write(pos); // move servo to 0 degrees
  Serial.println("Positioned at 0 Degrees");
  Serial.println("Input Desired Angle and Press Enter");
  /*while (!Serial);
  Serial.println("-------------------------");
  Serial.println("ARos is loading....");
  delay(1000);
  Serial.println("ARos loaded succesfully");
  Serial.println("-------------------------");
 
  Serial.println("calibrating servo...");
  for (pos = 0; pos <= 180; pos += 1)
    myservo.write(0);
  delay(1000);
  myservo.write(180);
  delay(1000);

  Serial.println("servo calibrated");
  Serial.println("-------------------------");
  Serial.println("Comand input online, write command to perform action");
  Serial.println("-------------------------");*/
}

void loop() {

while(Serial.available())


  {
    String comingdata = "";
    comingdata = Serial.readString();//the erray that will be recieved from processing
    Serial.print( comingdata );
    float state = comingdata.toFloat();//etikit boytu
    while(state >= 0 && state<180)
    {
      Serial.print(">");
      Serial.println(state);
      Serial.print("turning servo to ");
      Serial.print(state);
      Serial.println(" degrees");
        myservo.write(pos);
        Serial.print("+");
        Serial.println(pos);
        pos=pos+1;
        state=state-1;
       
        delay(dela);
    }
    while(state <= 0)
    {
      Serial.print(">");
      Serial.println(state);
      Serial.print("turning servo to ");
      Serial.print(state);
      Serial.println(" degrees");
        myservo.write(pos);
        Serial.print("+");
        Serial.println(pos);
        pos=pos-1;
        state=state+1;
        delay(dela);
    }

  }

}
