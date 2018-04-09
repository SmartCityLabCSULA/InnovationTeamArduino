// This is the code for slot car using H-bridge (L293D) & Teensy 3.2

// define variables

//IR sensor
//int sensorValue;
//float Distance;

// Ultrasonic Sensor
const int UTRASONIC_TRIG_PIN = 17;
const int UTRASONIC_ECHO_PIN = 16;
long duration;
float distance;

// H-bridge & motor Pins
int H_BRIDGE_INPUT_PIN_1 = 21;  // H-bridge leg 1 pin 3--> Teensy Pin 2
int H_BRIDGE_INPUT_PIN_2 = 22;  // H-bridge leg 2 pin 6--> Teensy Pin 3
int H_BRIDGE_ENABLE_PIN = 23;  // H-bridge enable Pin 1 --> Teensy Pin 23 x
//int pwmVal;

// LEDs
int LED_PIN_RED = 19;
int LED_PIN_GREEN = 20;
int LED_PIN_ON = 13;

// Baud rate
int BAUD_RATE = 9600;

int MOTOR_MIN_PWM = 0;
int MOTOR_MAX_PWM = 180;
double MIN_HEADWAY_DISTANCE = 3; // cm
double MAX_HEADWAY_DISTANCE = 10; //cm

// Speed of sound divided by two
double SPEED_OF_SOUND_2 = 343/2; // m/s

void setup() {
  // define distance to zero
  //Distance = 0;

  // define H-bridge pins
  pinMode(MAX_HEADWAY_DISTANCE, OUTPUT);
  pinMode(H_BRIDGE_INPUT_PIN_2, OUTPUT);
  pinMode(H_BRIDGE_ENABLE_PIN, OUTPUT);

  // digital write
  //digitalWrite(H_BRIDGE_ENABLE_PIN, HIGH);

  // H_BRIDGE_ENABLE_PIN --> will turn motor on
  digitalWrite(H_BRIDGE_ENABLE_PIN, HIGH);

  // Ultrasonic Sensor
  pinMode(UTRASONIC_TRIG_PIN, OUTPUT);
  pinMode(UTRASONIC_ECHO_PIN, INPUT);

  // LEDs
  pinMode(LED_PIN_RED, OUTPUT);
  pinMode(LED_PIN_GREEN, OUTPUT);
  pinMode(LED_PIN_ON, OUTPUT);

  // baud rate. May have to change the frequency...
  Serial.begin(BAUD_RATE);
  digitalWrite(LED_PIN_ON, HIGH);
}

void loop() {


  // Ultrasonic Sensor
  digitalWrite(UTRASONIC_TRIG_PIN, LOW);
  delayMicroseconds(2);

  // Sets the UTRASONIC_TRIG_PIN on HIGH state for 10 microseconds
  digitalWrite(UTRASONIC_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(UTRASONIC_TRIG_PIN, LOW);

  // read UTRASONIC_ECHO_PIN
  duration = pulseIn(UTRASONIC_ECHO_PIN, HIGH);

  // calculate the distance
  distance = duration*SPEED_OF_SOUND_2;

  // print
  //Serial.print("Distance ");
  //Serial.println(distance);

  ///////////////////////////////////////////////////////////////
  // motor --> PWM

  // write to each motor leg
  digitalWrite(H_BRIDGE_INPUT_PIN_1, HIGH);
  digitalWrite(H_BRIDGE_INPUT_PIN_2, LOW);

  int pwm = map(distance, MOTOR_MIN_PWM, MOTOR_MAX_PWM,
                MIN_HEADWAY_DISTANCE, MAX_HEADWAY_DISTANCE);
  analogWrite(H_BRIDGE_ENABLE_PIN, pwm);

  if (distance > MIN_HEADWAY_DISTANCE) {
    // Turn on green LED and turn off red LED if we're greater than
    // the minimum distance
    digitalWrite(LED_PIN_RED, LOW);
    digitalWrite(LED_PIN_GREEN, HIGH);
  } else {
    // Turn on red LED and turn off green LED if we're less than the
    // minimum distance
    digitalWrite(LED_PIN_RED, HIGH);
    digitalWrite(LED_PIN_GREEN, LOW);
  }

  Serial.print("Distance ");
  Serial.println(distance);
  //Serial.println("ON");
}
