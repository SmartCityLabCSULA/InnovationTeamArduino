int sensorPin = A0;   // Pin to read sensor values

void setup() {
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
}
