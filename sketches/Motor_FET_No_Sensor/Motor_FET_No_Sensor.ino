int enablePin = 9;    // Pin to output PWM values
const int ledPinG =  12;
const int ledPinY =  11;
int pwmVal;           // Variable to store PWM values




void setup() {
  // In the setup(), set pin 9 as an ouput pin
  // Set the enable pin high so the FET can 
  // turn the motor on.
  
  // set pinmodes for each pin:
  pinMode(enablePin, OUTPUT);
  pinMode(ledPinY, OUTPUT);
  pinMode(ledPinG, OUTPUT);
  // set enablePin HIGH so that motor starts off:
  digitalWrite(enablePin, HIGH);
    Serial.begin(9600); // start the serial port
}

void loop() {
  digitalWrite(ledPinY, HIGH);
  analogWrite (enablePin, 108);
  Serial.println("108");
  delay(5000);
  digitalWrite(ledPinY, LOW);
  digitalWrite(ledPinG, HIGH);
  analogWrite (enablePin, 115);
  Serial.println("115");
  delay(5000);
  digitalWrite(ledPinG, LOW);

  
  }
