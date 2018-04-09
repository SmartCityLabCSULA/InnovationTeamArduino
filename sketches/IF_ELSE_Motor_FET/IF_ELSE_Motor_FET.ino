int enablePin = 9;    // Pin to output PWM values
int pwmVal;           // Variable to store PWM values
int DistanceLOW = 0.8;
int DistanceHIGH = 0.25;

void setup() {
  // In the setup(), set pin 9 as an ouput pin
  // Set the enable pin high so the FET can 
  // turn the motor on.
  
  // set pinmodes for each pin:
  pinMode(enablePin, OUTPUT);
  // set enablePin low so that motor starts off:
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
  if(Distance >= .25){
    analogWrite(enablePin, 130);
  }
  else if (Distance >= .08 && Distance < .25){
    analogWrite(enablePin, 120);
  }
  else{
  analogWrite(enablePin, 0);
  }
  }
