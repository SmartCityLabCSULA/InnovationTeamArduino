int sensorPin = A0;
int motor1Pin = 3;    // H-bridge leg 1 (pin 2, 1A)
int motor2Pin = 4;    // H-bridge leg 2 (pin 7, 2A)
int enablePin = 9;    // H-bridge enable pin
int pwmVal;
int sensorVal;




void setup() {
  // In the setup(), set all the pins for the H-bridge 
  // as outputs.
  // Set the enable pin high so the H-bridge can 
  // turn the motor on.
  
  // set all the other pins you're using as outputs:
  pinMode(motor1Pin, OUTPUT);
  pinMode(motor2Pin, OUTPUT);
  pinMode(enablePin, OUTPUT);
  
  // set enablePin high so that motor can turn on:
  digitalWrite(enablePin, HIGH);
  
  Serial.begin(9600); // start the serial port
}

void loop() {

  // 5v
  float volts = analogRead(sensorPin)*0.0048828125;  // value from sensor * (5/1024)
  int distance = 13*pow(volts, -1); // worked out from datasheet graph
  delay(250); // slow down serial port
  Serial.println(distance);
  
  if (distance >= 7 ) {
    digitalWrite(motor1Pin, HIGH);   // set leg 1 of the H-bridge low
    digitalWrite(motor2Pin, LOW);  // set leg 2 of the H-bridge high 

    //digitalWrite(ledPin, HIGH);
    //read sensor value and set upper limit cap
  sensorVal = analogRead(sensorPin);
  if(sensorVal > 25){
    sensorVal = 25;
  }

  //map and assign pwm values to the motor output 0 to 255 corresponds to 0 to 100%
  pwmVal = map(sensorVal, 7, 25, 26, 255);

  //set 450 as out cutout or cut in limit where the fan switches from off to the lower PWM limit
  if(sensorVal < 7){
    pwmVal = 0;
  }

  //write the PWM value to the pwm output pin
  analogWrite(enablePin, pwmVal);
  } 
  
  else {
      digitalWrite(motor1Pin, LOW);   // set leg 1 of the H-bridge low
      digitalWrite(motor2Pin, LOW);  // set leg 2 of the H-bridge low 
      analogWrite(enablePin, 0); // Stop motor
  }
}
