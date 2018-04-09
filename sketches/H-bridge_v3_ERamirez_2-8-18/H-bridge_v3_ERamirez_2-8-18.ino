// This is the code for slot car using H-bridge (L293D) & Teensy 3.2 
// written by Edward Ramirez

// define variables

// IR sensor
int sensorValue;
float Distance;

//LEDs
int LEDPin = 13;

// H-bridge & motor Pins
int motorPin2 = 21;  // H-bridge leg 1 pin 3--> Teensy Pin 2
int motorPin7 = 22;  // H-bridge leg 2 pin 6--> Teensy Pin 3
int enablePin = 23;  // H-bridge enable Pin 1 --> Teensy Pin 23 x
//int pwmVal;


void setup() {
  // define distance to zero
  Distance = 0;

  // define H-bridge pins
  pinMode(motorPin2, OUTPUT);
  pinMode(motorPin7, OUTPUT);
  pinMode(enablePin, OUTPUT);

  // digital write
  digitalWrite(enablePin, HIGH); 

  // enablepin --> will turn motor on
  digitalWrite(enablePin, HIGH);

  // baud rate. May have to change the frequency...
  Serial.begin(9600);
  pinMode(LEDPin, OUTPUT);
  digitalWrite(LEDPin, HIGH);
}

void loop() {

  // read the input on analog pin 0:
  // set sensorValue = 0
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
  //pwmVal = map(sensorValue, 2, 10, 25, 255);

  ///////////////////////////////////////////////////////////////
  // motor --> PWM

  // write to each motor leg
  digitalWrite(motorPin2, HIGH);
  digitalWrite(motorPin7, LOW);

  // set 450 as cutout or cut in the limit
  // note: PWM need to be modified 
  
  if (Distance > 10) {
    //pwmVal = 0;
    Serial.println(Distance);
    analogWrite(enablePin, 80);
  }
  else if ( Distance > 9 && Distance <=10){
    analogWrite(enablePin, 70);
  }
  else if ( Distance > 8 && Distance <= 9){
    analogWrite(enablePin, 60);
  }
  else if (Distance > 7 && Distance <= 8){
    analogWrite(enablePin, 50);
  }
  else if (Distance > 6 && Distance <= 7){
    analogWrite(enablePin, 40);
  }
  else if (Distance > 5 && Distance <= 6){
    analogWrite(enablePin, 40);  
  }
  else if (Distance > 4 && Distance <= 5){
    analogWrite(enablePin, 40);
  }
   else if (Distance > 3 && Distance <= 4){
    analogWrite(enablePin, 40);
  }
  else {
    digitalWrite(motorPin2, LOW);
    digitalWrite(motorPin7, LOW);
    analogWrite(enablePin, 0);
  }
  // for printing
  Serial.print(Distance);
  Serial.println("");
  delay(250);
}

