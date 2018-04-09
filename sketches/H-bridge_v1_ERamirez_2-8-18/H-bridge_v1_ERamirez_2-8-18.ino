// This is the code for slot car using H-bridge (L293D)
// written by Edward Ramirez

// define variables

// IR sensor
int sensorValue;
float Distance;

// H-bridge, motor, & PWM
int motorPin2 = 21;  // H-bridge leg 1 pin 3--> Teensy Pin 2
int motorPin7 = 22;  // H-bridge leg 2 pin 6--> Teensy Pin 3
int enablePin = 23;  // H-bridge enable Pin 1 --> Teensy Pin 23 x
int pwmVal;


void setup() {
  // define distance to zero
  Distance = 0;

  // define H-bridge pins
  pinMode(motorPin2, OUTPUT);
  pinMode(motorPin7, OUTPUT);
  pinMode(enablePin, OUTPUT);

  // enablepin --> will turn motor on
  digitalWrite(enablePin, HIGH);

  // baud rate. May have to change the frequency...
  Serial.begin(9600);
}

void loop() {

  // read the input on analog pin 0:
  // set sensorValue = 0
  sensorValue = 0;
  // loop for sensor
  for (int x = 0; x < 16; x++) {
    sensorValue = sensorValue + analogRead(A1);
  }
  // shift the register
  sensorValue = sensorValue >> 4;
  // pow function for experimental data
  Distance = pow((sensorValue / 1895.9), -1.089);

  // loop for H-bridge
  pwmVal = map(sensorValue, 2, 10, 25, 255);

  // set 450 as cutout or cut in the limit
  if (Distance < 2) {
    //pwmVal = 0;
    Serial.println(Distance);
    analogWrite(enablePin, 0);
  }
  else if ( Distance <= 10) {
    analogWrite(enablePin, 80);
  }
  else if ( Distance <= 25) {
    analogWrite(enablePin, 105);
  }
  else if ( Distance <= 30) {
    analogWrite(enablePin, 120);
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

