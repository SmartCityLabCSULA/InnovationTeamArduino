// This is the code for slot car using H-bridge (L293D) & Teensy 3.2

// define variables

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

  // define H-bridge pins
  pinMode(motorPin2, OUTPUT);
  pinMode(motorPin7, OUTPUT);
  pinMode(enablePin, OUTPUT);

  // enablepin --> will turn motor on
  digitalWrite(enablePin, HIGH);

  // LEDs
  pinMode(ledPinR, OUTPUT);
  pinMode(ledPinG, OUTPUT);
  pinMode(ledPinON, OUTPUT);

  // baud rate. May have to change the frequency...
  Serial.begin(9600);
  digitalWrite(ledPinON, HIGH);
}

void loop() {
  
  // motor --> PWM

  // write to each motor leg
  digitalWrite(motorPin2, HIGH);
  digitalWrite(motorPin7, LOW);
  digitalWrite(ledPinR, LOW);
  digitalWrite(ledPinG, HIGH);
  analogWrite(enablePin, 150);
  Serial.print("PWM ");
  Serial.println("150");
  delay(5000);
  digitalWrite(motorPin2, LOW);
  digitalWrite(motorPin7, LOW);
  digitalWrite(ledPinR, HIGH);
  digitalWrite(ledPinG, LOW);
  analogWrite(enablePin, 0);
  Serial.print("PWM ");
  Serial.println("0");
  delay(5000);
  digitalWrite(motorPin2, HIGH);
  digitalWrite(motorPin7, LOW);
  digitalWrite(ledPinR, LOW);
  digitalWrite(ledPinG, HIGH);
  analogWrite(enablePin, 255);
  Serial.print("PWM ");
  Serial.println("255");
  delay(5000);
  digitalWrite(motorPin2, LOW);
  digitalWrite(motorPin7, LOW);
  digitalWrite(ledPinR, HIGH);
  digitalWrite(ledPinG, LOW);
  analogWrite(enablePin, 0);
  Serial.print("PWM ");
  Serial.println("0");
  delay(5000);
}

