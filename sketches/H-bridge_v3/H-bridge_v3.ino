// This is the code for slot car using H-bridge (L293D) & Teensy 3.2
// written by Edward Ramirez

// define variables

// IR sensor
//int sensorValue;
//float Distance;

// Ultrasonic Sensor
const int trigPin = 14;
const int echoPin = 12;
long duration;
float distance;

// H-bridge & motor Pins
int motorPin2 = 21;  // H-bridge leg 1 pin 3--> Teensy Pin 2
int motorPin7 = 22;  // H-bridge leg 2 pin 6--> Teensy Pin 3
int enablePin = 23;  // H-bridge enable Pin 1 --> Teensy Pin 23 x
//int pwmVal;

// LEDs
int ledPinR = 19;
int ledPinG = 20;
int ledPinON = 13;


void setup() {
  // define distance to zero
  //Distance = 0;

  // define H-bridge pins
  pinMode(motorPin2, OUTPUT);
  pinMode(motorPin7, OUTPUT);
  pinMode(enablePin, OUTPUT);

  // digital write
  //digitalWrite(enablePin, HIGH);

  // enablepin --> will turn motor on
  digitalWrite(enablePin, HIGH);

  // Ultrasonic Sensor
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // LEDs
  pinMode(ledPinR, OUTPUT);
  pinMode(ledPinG, OUTPUT);
  pinMode(ledPinON, OUTPUT);

  // baud rate. May have to change the frequency...
  Serial.begin(9600);
  digitalWrite(ledPinON, HIGH);
}

void loop() {

  // read the input on analog pin 0:
  // set sensorValue = 0
  /*
    sensorValue = 0;
    // loop for sensor
    for (int x = 0; x < 16; x++) {
    sensorValue = sensorValue + analogRead(A0);
    }
    // shift the register
    sensorValue = sensorValue >> 4;
    // pow function for experimental data
    Distance = pow((sensorValue / 1895.9), -1.089);

    // loop for H-bridge
    //pwmVal = map(sensorValue, 2, 10, 25, 255)
  */

  // Ultrasonic Sensor
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  // Sets the tripin on HIGH state for 10 microseconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // read echoPin
  duration = pulseIn(echoPin, HIGH);

  // calculate the distance
  distance = duration*0.034/2;

  // print
  //Serial.print("Distance ");
  //Serial.println(distance);



  ///////////////////////////////////////////////////////////////
  // motor --> PWM

  // write to each motor leg
  digitalWrite(motorPin2, HIGH);
  digitalWrite(motorPin7, LOW);

  // note: PWM need to be modified

  if (distance > 10) {
    //pwmVal = 0;
    Serial.println(distance);
    digitalWrite(ledPinR, LOW);
    digitalWrite(ledPinG, HIGH);
    analogWrite(enablePin, 180);
  }
  else if ( distance > 9 && distance <= 10) {
    digitalWrite(ledPinR, LOW);
    digitalWrite(ledPinG, HIGH);
    analogWrite(enablePin, 160);
  }
  else if ( distance > 8 && distance <= 9) {
    digitalWrite(ledPinR, LOW);
    digitalWrite(ledPinG, HIGH);
    analogWrite(enablePin, 140);
  }
  else if (distance > 7 && distance <= 8) {
    digitalWrite(ledPinR, LOW);
    digitalWrite(ledPinG, HIGH);
    analogWrite(enablePin, 130);
  }
  else if (distance > 6 && distance <= 7) {
    digitalWrite(ledPinR, LOW);
    digitalWrite(ledPinG, HIGH);
    analogWrite(enablePin, 100);
  }
  else if (distance > 5 && distance <= 6) {
    digitalWrite(ledPinR, LOW);
    digitalWrite(ledPinG, HIGH);
    analogWrite(enablePin, 60);
  }
  else if (distance > 4 && distance <= 5) {
    digitalWrite(ledPinR, LOW);
    digitalWrite(ledPinG, HIGH);
    analogWrite(enablePin, 45);
  }
  else if (distance > 3 && distance <= 4) {
    digitalWrite(ledPinR, LOW);
    digitalWrite(ledPinG, HIGH);
    analogWrite(enablePin, 0);
  }
  else {
    digitalWrite(ledPinG, LOW);
    digitalWrite(ledPinR, HIGH);
    digitalWrite(motorPin2, LOW);
    digitalWrite(motorPin7, LOW);
    analogWrite(enablePin, 0);
  }
  // for printing
  Serial.print("Distance ");
  Serial.println(distance);
  //Serial.println("ON");
}

