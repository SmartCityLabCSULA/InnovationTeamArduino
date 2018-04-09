int sensorPin = A0;   // Pin to read sensor values
int enablePin = 9;    // Pin to output PWM values
int pwmVal;           // Variable to store PWM values




void setup() {
  // In the setup(), set pin 9 as an ouput pin
  // Set the enable pin high so the H-bridge can 
  // turn the motor on.
  
  // set pinmodes for each pin:
  pinMode(enablePin, OUTPUT);
  pinMode(sensorPin, INPUT);
  // set enablePin high so that motor can turn on:
  digitalWrite(enablePin, HIGH);
  
  Serial.begin(9600); // start the serial port
}

void loop() {

  int sensorValue = 0;
  for(int x = 0; x < 100; x++){
    sensorValue = sensorValue + analogRead(A0);
  }
  float Distance = pow((sensorValue/1923.2), -0.914);
  delay(250); // slow down serial port
  Serial.println(Distance);
  
  if (Distance >= 7 ) {
  //read sensor value and set upper limit cap  
  Distance = analogRead(sensorPin); 
  if(Distance > 25){
    pwmVal = 200;
  //map and assign pwm values to the motor output 0 to 255 corresponds to 0 to 100%
  pwmVal = map(Distance, 7, 25, 0, 200);
  }
  //set 7 as cutoff or cut in limit where the fan switches from off to the lower PWM limit
  if(Distance < 7){
    pwmVal = 0;
  }

  //write the PWM value to the pwm output pin
  analogWrite(enablePin, pwmVal);
  } 
  
  else {
      analogWrite(enablePin, 0); // Stop motor
  }
}
