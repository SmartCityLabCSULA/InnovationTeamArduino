#include <cmath>
#include <XBee.h>
#include <Wire.h> // Must include Wire library for I2C
#include <SparkFun_MMA8452Q.h> // Includes the SFE_MMA8452Q library
#include <human_driver.h>

// Constants definitions
// Accelerometer constants
const double GRAV_CONST = 9.8;
const int SDA_PIN = 18;
const int SCL_PIN = 19;
const bool ACCEL_FIRST_READ = false;
const int ACCEL_CALIB_READ_COUNTS = 10;
const int ACCEL_DEADBAND = 1.0;

// End-point constants
const int CAR_ID = 3;

// LED constant
const int LED_PIN_RED = 8;
const int LED_PIN_GREEN = 9;
const int LED_PIN_ON = 13;

// Baud rate
const int BAUD_RATE = 115200;

// H-bridge & motor Pins
const int H_BRIDGE_INPUT_PIN_1 = 21; // Teensy Pin 21
const int H_BRIDGE_INPUT_PIN_2 = 22; // Teensy Pin 22
const int H_BRIDGE_ENABLE_PIN = 23;  // Teensy Pin 23

// Conversion constants
// Scale to convert velocity to PWM
const double SPEED_OF_SOUND_2 = 0.0343 / 2; // cm/s (NOTE: is this right?)

// Other constants
const double REL_VEL_DEADBAND = 0.1;

// Global variable definitions
MMA8452Q accel;

double accel_y = 0.0;
double accel_y_offset = 0.0;

bool accel_calibration_complete = false;
int accel_calib_counts = 0;

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

double duration = 0.0;
double current_headway = 0.0;
double previous_headway = 0.0;
double relative_velocity = 0.0;
double current_velocity = 0.0;
double previous_velocity = 0.0;
double previous_time = 0.0;
double start_time = 0.0;
double current_time = 0.0;
double delta_time = 0.0;

int pwm_value = 0;

void setup() {
  // Init  accelerometer
  Wire.setSDA(SDA_PIN);
  Wire.setSCL(SCL_PIN);
  accel.init(SCALE_8G, ODR_800);

  // define H-bridge pins
  pinMode(H_BRIDGE_INPUT_PIN_1, OUTPUT);
  pinMode(H_BRIDGE_INPUT_PIN_2, OUTPUT);
  pinMode(H_BRIDGE_ENABLE_PIN, OUTPUT);

  // H_BRIDGE_ENABLE_PIN --> will turn motor on
  digitalWrite(H_BRIDGE_ENABLE_PIN, HIGH);

  // baud rate. May have to change the frequency...
  Serial.begin(BAUD_RATE);
  Serial1.begin(BAUD_RATE);//configured teensy ports for communication
  xbee.setSerial(Serial1);
  digitalWrite(LED_PIN_ON, HIGH);
  start_time = (double) millis();
}

void loop() {

  current_time = (double) millis() - start_time;
  delta_time = std::abs((current_time - previous_time) / 1000);
  if (accel.available()) {
    // First, use accel.read() to read the new variables:
    accel.read();
    if (!accel_calibration_complete) {
      // HACK
      // There seems to be some kind of bias so we remove it
      accel_y_offset += accel.cy;
      accel_calib_counts++;

      if (accel_calib_counts == ACCEL_CALIB_READ_COUNTS) {
        accel_calibration_complete = true;
      }
    } else {
      accel_y = (accel.cy - accel_y_offset) * GRAV_CONST;

      if (std::abs(accel_y) < ACCEL_DEADBAND) {
        accel_y = 0.0;
      }

      current_velocity += accel_y * delta_time;
      previous_velocity = current_velocity;
    }
  }

  payload[0] = CAR_ID;//Each car had a unique ID assigned
  payload[1] = 1;//Scenario 1: Stop and Go Waves
  ReceiveData();

  // calculate the distance
  current_headway = duration * SPEED_OF_SOUND_2;
  relative_velocity = (current_headway - previous_headway) / delta_time;
  if (std::abs(relative_velocity) < REL_VEL_DEADBAND)
  {
      relative_velocity = 0.0;
  }

  payload[3] = current_headway;

  ReceiveData();

  digitalWrite(H_BRIDGE_INPUT_PIN_1, HIGH);
  digitalWrite(H_BRIDGE_INPUT_PIN_2, LOW);

  analogWrite(H_BRIDGE_ENABLE_PIN, pwm_value);
  payload[2]=pwm_value;
  ReceiveData();

  ReceiveData();
  previous_headway = current_headway;
  previous_time = current_time;
  Serial.print("V:\t"); Serial.print(current_velocity); Serial.print("\t");
  Serial.print("A:\t"); Serial.print(accel_y); Serial.print("\t");
  Serial.print("ct:\t"); Serial.print(current_time); Serial.print("\t");
  Serial.print("pt:\t"); Serial.print(previous_time); Serial.print("\t");
  Serial.print("dt:\t"); Serial.print(delta_time); Serial.print("\t");
  Serial.print("A bias:\t"); Serial.println(accel_y_offset);
}


void ReceiveData() {
  int car_id_received = 0;
  xbee.send(tx);
  xbee.readPacket(10);
  if (xbee.getResponse().isAvailable()) {
    if (xbee.getResponse().getApiId() == RX_16_RESPONSE) {
      xbee.getResponse().getRx16Response(rx16);
      car_id_received = rx16.getData(1);
    }
  }

  if (car_id_received == CAR_ID) {
    for (int j = 0; j < 5; j++) {
      xbee.send(tx);
    }
  }
}
