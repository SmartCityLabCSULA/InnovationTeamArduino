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
bool x = true;
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

  // Ultrasonic Sensor
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);
  // Calculating the distance
  distance = (duration * 0.034) / 2;
  delay(150);
  // Prints the distance on the Serial Monitor
  //Serial.print("Distance: ");
  time = time + 0.02;

  Serial.print(distance);
  Serial.print("\t");
  Serial.println(time);



  ///////////////////////////////////////////////////////////////
  // motor --> PWM

  // write to each motor leg
  digitalWrite(motorPin2, HIGH);
  digitalWrite(motorPin7, LOW);

  // note: PWM need to be modified

  if (distance > 12) {
    //pwmVal = 0;
    digitalWrite(ledPinR, LOW);
    digitalWrite(ledPinG, HIGH);
    analogWrite(enablePin, 180);
  }
  else if (distance <= 12) {
    digitalWrite(ledPinG, LOW);
    digitalWrite(ledPinR, HIGH);
    analogWrite(enablePin, 80);
    digitalWrite(motorPin2, LOW);
    digitalWrite(motorPin7, HIGH);
    //delay(300);
    Serial.print(distance);
    Serial.print("\t");
    while (distance <= 12) {
      // Ultrasonic Sensor
      digitalWrite(trigPin, LOW);
      delayMicroseconds(2);
      // Sets the trigPin on HIGH state for 10 micro seconds
      digitalWrite(trigPin, HIGH);
      delayMicroseconds(10);
      digitalWrite(trigPin, LOW);
      // Reads the echoPin, returns the sound wave travel time in microseconds
      duration = pulseIn(echoPin, HIGH);
      // Calculating the distance
      distance = (duration * 0.034) / 2;
      // Prints the distance on the Serial Monitor
      //Serial.print("Distance: ");
      time = time + 0.02;
      Serial.print(distance);
      Serial.print("\n");
      analogWrite(enablePin, 0);
      digitalWrite(motorPin2, LOW);
      digitalWrite(motorPin7, LOW);
      analogWrite(enablePin, 0);
    Serial.print("out of while");
    Serial.print("\n");
  }
}
}

