#include <range_policy.h>
#include <XBee.h>
// This is the code for slot car using H-bridge (L293D) & Teensy 3.2

//XBee configuration
XBee xbee = XBee();
// allocate two bytes for to hold a 10-bit analog reading
uint8_t payload[] = { 0, 0, 0, 0 }; //data to be transmitted: Car ID, Scenario, Velocity, Headway
XBeeAddress64 addr64 = XBeeAddress64(0x0013a200, 0x4106a272);
Tx64Request tx = Tx64Request(addr64, payload, sizeof(payload));
XBeeResponse response = XBeeResponse();
// create reusable response objects for responses we expect to handle
Rx16Response rx16 = Rx16Response();
Rx64Response rx64 = Rx64Response();

// define variables

int CAR_ID = 3;
int SCENARIO_RECEIVED;
int CAR_ID_RECEIVED;//Used for the filter
int j;
//IR sensor
//int sensorValue;
//float Distance;

// Ultrasonic Sensor
const int UTRASONIC_TRIG_PIN = 14;
const int UTRASONIC_ECHO_PIN = 15;
long duration = 0.0;
float headway = 0.0;
double velocity = 0.0;

// H-bridge & motor Pins
const int H_BRIDGE_INPUT_PIN_1 = 21;  // H-bridge leg 1 pin 3--> Teensy Pin 2
const int H_BRIDGE_INPUT_PIN_2 = 22;  // H-bridge leg 2 pin 6--> Teensy Pin 3
const int H_BRIDGE_ENABLE_PIN = 23;  // H-bridge enable Pin 1 --> Teensy Pin 23 x
//int pwmVal;

// LEDs
const int LED_PIN_RED = 19;
const int LED_PIN_GREEN = 20;
const int LED_PIN_ON = 13;

// Baud rate
const int BAUD_RATE = 115200;



const int MOTOR_MIN_PWM = 2;
const int MOTOR_MAX_PWM = 150;
const double MIN_HEADWAY_DISTANCE = 15; // cm
const double MAX_HEADWAY_DISTANCE = 40; //cm
const double MIN_VELOCITY = 0.0; // cm/s
const double MAX_VELOCITY = 10.0; // cm/s

// Scale to convert velocity to PWM
const int VELOCITY_TO_PWM = MOTOR_MAX_PWM / MAX_VELOCITY;

// Speed of sound divided by two
const double SPEED_OF_SOUND_2 = 0.0343 / 2; // cm/s

RangePolicy range_policy(MIN_HEADWAY_DISTANCE, MAX_HEADWAY_DISTANCE, MAX_VELOCITY);
int pwm_value;

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
  Serial1.begin(115200);//configured teensy ports for communication
  xbee.setSerial(Serial1);
  digitalWrite(LED_PIN_ON, HIGH);
}

void loop() {
  payload[0] = CAR_ID;//Each car had a unique ID assigned
  payload[1] = 1;//Scenario 1: Stop and Go Waves
  ReceiveData();

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
  headway = duration * SPEED_OF_SOUND_2;
  payload[3] = headway;
  ReceiveData();
  // print
  //Serial.print("Distance ");
  //Serial.println(distance);

  ///////////////////////////////////////////////////////////////
  // motor --> PWM

  // write to each motor leg
  digitalWrite(H_BRIDGE_INPUT_PIN_1, HIGH);
  digitalWrite(H_BRIDGE_INPUT_PIN_2, LOW);

  // Determine the velocity via the range policy and then determine what
  // PWM value that corresponds to
  velocity = range_policy.velocity(headway);
  //  pwm_value = map(headway,
  //                MIN_VELOCITY, MAX_VELOCITY,
  //                MOTOR_MIN_PWM, MOTOR_MAX_PWM);
  pwm_value = VELOCITY_TO_PWM * velocity;
  analogWrite(H_BRIDGE_ENABLE_PIN, pwm_value);
  payload[2]=pwm_value;
  ReceiveData();

  // Logic for using a braking mechanism. Don't try this yet, it makes the range
  // policy not differentiable
  // if (velocity >= 0){
  //   pwm_value = map(headway, MOTOR_MIN_PWM, MOTOR_MAX_PWM,
  //                 MIN_VELOCITY, MAX_VELOCITY);
  //   analogWrite(H_BRIDGE_ENABLE_PIN, pwm_value);
  // } else {
  //   pwm_value = BRAKING_PWM;
  //   digitalWrite(H_BRIDGE_INPUT_PIN_1, LOW);
  //   digitalWrite(H_BRIDGE_INPUT_PIN_2, HIGH);
  //   analogWrite(H_BRIDGE_ENABLE_PIN, pwm_value)
  // }

  if (headway > MIN_HEADWAY_DISTANCE) {
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

  Serial.print("Distance :\t");
  Serial.print(headway);
  Serial.print("\tPWM:\t");
  Serial.println(pwm_value);

  //Serial.println("ON");
  ReceiveData();
}
void ReceiveData() {
  xbee.send(tx);
  xbee.readPacket(10);
  if (xbee.getResponse().isAvailable()) {
    if (xbee.getResponse().getApiId() == RX_16_RESPONSE) {
      xbee.getResponse().getRx16Response(rx16);
      CAR_ID_RECEIVED = rx16.getData(1);
      //scenario = rx16.getData(0);
    }
//    if (scenario == 0) {
//      contS1 = false;
//    }
  }
  if (CAR_ID_RECEIVED == CAR_ID) {
    for (j = 0; j < 5; j++) {
      xbee.send(tx);
    }
  }
}
