int enablePin = 23;    // Pin to output PWM values
int pwmVal;           // Variable to store PWM values
// defines pins numbers
const int trigPin = 21;
const int echoPin = 22;
const int LEDOn = 13;
// defines variables
long duration;
int distance;
// time
unsigned long time;

void setup() {
  // In the setup(), set pin 9 as an ouput pin
  // Set the enable pin high so the FET can 
  // turn the motor on.
  
  // set pinmodes for each pin:
  pinMode(enablePin, OUTPUT);
  // set enablePin high so that motor starts off:
  //digitalWrite(enablePin, HIGH);
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input
  time = 0;
  Serial.begin(9600); // start the serial port
  digitalWrite(LEDOn, HIGH);
}

void loop() {
  
 // int sensorValue = 0;
 // for(int x = 0; x < 100; x++){
 //   sensorValue = sensorValue + analogRead(A0);
 // }
  //float Distance = pow((sensorValue/1923.2), -0.914);
  //delay(250); // slow down serial port
  // Clears the trigPin
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

//Serial.print(distance);
//Serial.print("\t");
//Serial.println(time);
  //Serial.println(Distance);
  if (distance >= 25){
    pwmVal = 115;
    Serial.println(">115");
  }
  
  if (distance >= 6 && distance < 25){
    pwmVal = 115 * (distance/20);
    analogWrite(enablePin, pwmVal);
    Serial.print(pwmVal);
  }
  else if (distance < 6){
  analogWrite(enablePin, 0);
  Serial.println("0");
 //rrr   Serial.println('\n');
  }
  }
