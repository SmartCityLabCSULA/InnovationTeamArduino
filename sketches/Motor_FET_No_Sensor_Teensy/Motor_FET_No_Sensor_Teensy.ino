int enablePin = A8;    // Pin to output PWM values
int pwmVal;           // Variable to store PWM values
const int ledPin =  13;      // the number of the LED pin




void setup() {
  // In the setup(), set pin 9 as an ouput pin
  // Set the enable pin high so the FET can 
  // turn the motor on.
  
  // set pinmodes for each pin:
  pinMode(enablePin, OUTPUT);
  // set enablePin HIGH so that motor starts off:
  digitalWrite(enablePin, HIGH);
  pinMode(ledPin, OUTPUT);
    Serial.begin(9600); // start the serial port
}

void loop() {
  digitalWrite(ledPin, HIGH);
  analogWrite (enablePin, 87);
  Serial.println("87");
  delay(5000);
  analogWrite (enablePin, 105);
  Serial.println("105");
  delay(5000);

  
  }
