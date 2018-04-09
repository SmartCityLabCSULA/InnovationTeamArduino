// This is the code for slot car using H-bridge (L293D) & Teensy 3.2

// define variables

// Ultrasonic Sensor
const int trigPin = 14;
const int echoPin = 15;
long duration;
float distance;
unsigned long time;

// LEDs
int ledPinR = 19;
int ledPinG = 20;
int ledPinON = 13;


void setup() {

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
distance= duration*0.034/2;
delay(150);
// Prints the distance on the Serial Monitor
//Serial.print("Distance: ");
time = time + 0.02;

Serial.print(distance);
Serial.print("\t");
Serial.println(time);



  ///////////////////////////////////////////////////////////////

  // note: PWM need to be modified

  if (distance > 10) {
    //pwmVal = 0;
    digitalWrite(ledPinR, LOW);
    digitalWrite(ledPinG, HIGH);
  }
  else if ( distance > 9 && distance <= 10) {
    digitalWrite(ledPinR, LOW);
    digitalWrite(ledPinG, HIGH);
  }
  else if ( distance > 8 && distance <= 9) {
    digitalWrite(ledPinR, LOW);
    digitalWrite(ledPinG, HIGH);
  }
  else if (distance > 7 && distance <= 8) {
    digitalWrite(ledPinR, LOW);
    digitalWrite(ledPinG, HIGH);
  }
  else if (distance > 6 && distance <= 7) {
    digitalWrite(ledPinR, LOW);
    digitalWrite(ledPinG, HIGH);
  }
  else if (distance > 5 && distance <= 6) {
    Serial.println(distance);
    digitalWrite(ledPinR, LOW);
    digitalWrite(ledPinG, HIGH);
  }
  else if (distance > 4 && distance <= 5) {
    digitalWrite(ledPinR, LOW);
    digitalWrite(ledPinG, HIGH);
  }
  else {
    digitalWrite(ledPinG, LOW);
    digitalWrite(ledPinR, HIGH);
  }
}

