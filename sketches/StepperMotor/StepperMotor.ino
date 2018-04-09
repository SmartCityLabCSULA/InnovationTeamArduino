#include <AccelStepper.h>
#define sensor A0 // Sharp IR GP2Y0A41SK0F (4-30cm, analog)

/*
Assumed Pin Mappings
| ULN2003 Board | Motor Phase | Arduino Pin |
| ------------- | ----------- | ----------- |
| IN1           | Blue        |           2 |
| IN2           | Pink        |           3 |
| IN3           | Yellow      |           4 |
| IN4           | Orange      |           5 |
*/



//AccelStepper stepper; // Defaults to AccelStepper::FULL4WIRE (4 pins) on 2, 3, 4, 5

//The first 4 means FULLSTEP, then the coil end pins Blue, Yellow, Pink, Orange
AccelStepper stepper(4, 2, 4, 3, 5);  

void setup()
{  
   stepper.setMaxSpeed(9000);
   
   //On my 28BYJ-48 with a 64:1 gear ratio my max speed was about 650
   //Positive speeds are clockwise, Negative Speeds are counter-clockwise
   stepper.setSpeed(650);
   Serial.begin(9600);   
}

void loop()
{  
   float volts = analogRead(sensor)*0.0048828125;  // value from sensor * (5/1024)
   int distance = 13*pow(volts, -1); // worked out from datasheet graph
   delay(250); // slow down serial port
   Serial.println(on);
   
   if (distance <= 10){
      Serial.println(distance);   // print the distance
      analogWrite(enablePin, 0); // Run in half speed
    } else if (distance <= 20 ) {
      analogWrite(enablePin, 135); // Run in half speed
    } else {
      analogWrite(enablePin, 255); // Run in full speed
    }
  } else { 
   stepper.runSpeed();
}
