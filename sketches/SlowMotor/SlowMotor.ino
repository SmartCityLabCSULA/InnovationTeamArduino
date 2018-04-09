// This is for the motor

const int switchPin = 2;    // Switch input
const int ledPin = 13;      // LED light
const int motor1Pin = 3;    // H-bridge leg 1 (pin 2, 1A)
const int motor2Pin = 4;    // H-bridge leg 2 (pin 7, 2A)
const int enablePin = 9;    // H-bridge enable pin
int on = false;
// This is for the sensor

#define sensor A0 // Sharp IR GP2Y0A41SK0F (4-30cm, analog)


void setup() {
  // In the setup(), set all the pins for the H-bridge 
  // as outputs, and the pin for the switch as an input.
  // The set the enable pin high so the H-bridge can 
  // turn the motor on.
  
  // set all the other pins you're using as outputs:
  pinMode(motor1Pin, OUTPUT);
  pinMode(motor2Pin, OUTPUT);
  pinMode(enablePin, OUTPUT);
  pinMode(switchPin, INPUT);
  pinMode(ledPin, OUTPUT);
  
  // set enablePin high so that motor can turn on:
  digitalWrite(enablePin, HIGH);
  
  Serial.begin(9600); // start the serial port
}

void loop() {

  // 5v
  float volts = analogRead(sensor)*0.0048828125;  // value from sensor * (5/1024)
  int distance = 13*pow(volts, -1); // worked out from datasheet graph
  delay(250); // slow down serial port
  
  int switchInput = digitalRead(switchPin);
  
  if (switchInput == HIGH) {
    Serial.println(on);   // print the distance
    on = !on;
  }
  
  if (on) {
    digitalWrite(motor1Pin, HIGH);   // set leg 1 of the H-bridge low
    digitalWrite(motor2Pin, LOW);  // set leg 2 of the H-bridge high 

    digitalWrite(ledPin, HIGH);
    if (distance <= 10){
      Serial.println(distance);   // print the distance
      analogWrite(enablePin, 0); // Run in half speed
    } else if (distance <= 20 ) {
      analogWrite(enablePin, 135); // Run in half speed
    } else {
      analogWrite(enablePin, 255); // Run in full speed
    }
  } else {
      digitalWrite(ledPin, LOW);
      digitalWrite(motor1Pin, LOW);   // set leg 1 of the H-bridge low
      digitalWrite(motor2Pin, LOW);  // set leg 2 of the H-bridge high 
      analogWrite(enablePin, 0); // Run in full speed
  }
}
