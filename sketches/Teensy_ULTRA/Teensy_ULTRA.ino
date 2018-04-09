// This is the code for slot car using H-bridge (L293D) & Teensy 3.2

#include <Ping.h>

// define variables

// Ultrasonic Sensor

Ping ping = Ping(18);  // Ping signal on pin 13

// LEDs
int ledPinR = 16;
int ledPinG = 17;
int ledPinON = 13;


void setup() {

  // LEDs
  pinMode(ledPinR, OUTPUT);
  pinMode(ledPinG, OUTPUT);
  pinMode(ledPinON, OUTPUT);

  // baud rate. May have to change the frequency...
  Serial.begin(115200);
  digitalWrite(ledPinON, HIGH);
}

void loop() {
ping.fire();
  Serial.print("Microseconds: ");
  Serial.print(ping.microseconds());
  Serial.print(" | Inches ");
  Serial.print(ping.inches());
  Serial.print(" | Centimeters: ");
  Serial.print(ping.centimeters());
  Serial.println();
  
  ///////////////////////////////////////////////////////////////

  // note: PWM need to be modified

  if (ping.centimeters() > 10) {
    //pwmVal = 0;
    digitalWrite(ledPinR, LOW);
    digitalWrite(ledPinG, HIGH);
  }
  else if ( ping.centimeters() > 9 && ping.centimeters() <= 10) {
    digitalWrite(ledPinR, LOW);
    digitalWrite(ledPinG, HIGH);
  }
  else if ( ping.centimeters() > 8 && ping.centimeters() <= 9) {
    digitalWrite(ledPinR, LOW);
    digitalWrite(ledPinG, HIGH);
  }
  else if (ping.centimeters() > 7 && ping.centimeters() <= 8) {
    digitalWrite(ledPinR, LOW);
    digitalWrite(ledPinG, HIGH);
  }
  else if (ping.centimeters() > 6 && ping.centimeters() <= 7) {
    digitalWrite(ledPinR, LOW);
    digitalWrite(ledPinG, HIGH);
  }
  else if (ping.centimeters() > 5 && ping.centimeters() <= 6) {
    digitalWrite(ledPinR, LOW);
    digitalWrite(ledPinG, HIGH);
  }
  else if (ping.centimeters() > 4 && ping.centimeters() <= 5) {
    digitalWrite(ledPinR, LOW);
    digitalWrite(ledPinG, HIGH);
  }
  else {
    digitalWrite(ledPinG, LOW);
    digitalWrite(ledPinR, HIGH);
  }
}

