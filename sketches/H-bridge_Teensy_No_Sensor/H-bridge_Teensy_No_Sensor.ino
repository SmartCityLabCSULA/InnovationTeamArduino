// This is the code for slot car using H-bridge (L293D) & Teensy 3.2

// define variables

// IR sensor
//int sensorValue;
//float Distance;

// Ultrasonic Sensor
const int trigPin = 14;
const int echoPin = 15;
long duration;
float distance;
unsigned long time;

// H-bridge & motor Pins
int motorPin2 = 21;  // H-bridge leg 1 pin 2--> Teensy Pin 21
int motorPin7 = 22;  // H-bridge leg 2 pin 7--> Teensy Pin 22
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

  // baud rate. May have to change the frequency...
  Serial.begin(9600);
  digitalWrite(ledPinON, HIGH);
}

void loop() {

  ///////////////////////////////////////////////////////////////
  // motor --> PWM

  // write to each motor leg
  digitalWrite(motorPin2, HIGH);
  digitalWrite(motorPin7, LOW);

  // note: PWM need to be modified

    digitalWrite(ledPinR, LOW);
    digitalWrite(ledPinG, HIGH);
    analogWrite(enablePin, 200);
    Serial.print("100");
    delay(5000);
    digitalWrite(ledPinG, LOW);
    digitalWrite(ledPinR, HIGH);
    digitalWrite(motorPin2, HIGH);
    digitalWrite(motorPin7, HIGH);
    analogWrite(enablePin, 200);
    Serial.print("70");
    delay(5000);
  }

