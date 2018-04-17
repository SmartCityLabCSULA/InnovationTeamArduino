//Teensy Coordinator: This set-up is used to select a scenario and get data
//Update 3/13/18: applied a filter to select which car is to send data and will
//only print when the correct Vehicle sent the data if not it will continue to
//scan until it does
#include <Printers.h>
#include <XBee.h>

///////////XBee Configuration///////////////
XBee xbee = XBee();
uint8_t payload[] = { 0, 0, 0, 0 }; //Car ID, scenario, velocity,distance, braking
XBeeAddress64 addr1 = XBeeAddress64(0x0013a200, 0x412727c9);
XBeeAddress64 addr2 = XBeeAddress64(0x0013a200, 0x4167d261);
XBeeAddress64 addr3 = XBeeAddress64(0x0013a200, 0x4106a2b4);
Tx64Request tx1 = Tx64Request(addr1, payload, sizeof(payload));
Tx64Request tx2 = Tx64Request(addr2, payload, sizeof(payload));
Tx64Request tx3 = Tx64Request(addr3, payload, sizeof(payload));

XBeeResponse response = XBeeResponse();
// create reusable response objects for responses we expect to handle
Rx16Response rx16 = Rx16Response();
Rx64Response rx64 = Rx64Response();
////////////////XBee Configuration End///////
int i;
int j;
int scenario;
int scenario1;
int velocity;
int distance;
int CarID = 3;
int Ton = 13; //teensy led pin
//int s1 = 19; //scenario 1 LED stop and go waves active
bool cont = true;
bool go = true;

int noc = 1; //number of cars

void setup() {
  Serial1.begin(115200);//configured teensy ports for communication
  Serial.begin(115200);//initializes baud rate for the rest of the ports
  xbee.setSerial(Serial1);
  //pinMode(s1, OUTPUT);
  pinMode(Ton, OUTPUT);
}
void loop() {
  digitalWrite(Ton, HIGH);//signifies teensy is on
  //delay(10);//necessary for teensy
  Serial.println("Enter Scenario");
  if (Serial.available()) {
    scenario = Serial.read(); //select the scenario
  }

  delay(500);

  switch (scenario) {
    case '1':
      payload[0] = 1;//scenario selection
      Serial.println("Stop and Go Waves");
      for (j = 0; j < 10; j++) {
        xbee.send(tx1);
        xbee.send(tx2);
        xbee.send(tx3);
      }
      Serial.println("Command Sent");
      cont = true;
     // digitalWrite(s1, HIGH);
      StopGo();
      payload[0] = 0;
      for (j = 0; j < 10; j++) {
        xbee.send(tx1);
        xbee.send(tx2);
        xbee.send(tx3);
      }
      delay(2000);
      //digitalW e(s1, LOW);
      scenario = 0;
      break;
    default:
      break;

  }
}
void StopGo() {
  delay(10);
  Serial.println("Stop and Go Wave Scenario Started");
  Serial.println();
  Serial.printf("Car ID, Scenario, Velocity, Distance \n");

  while (cont == true) {
    //Serial.println("Program Running");
    for (i = 1; i < noc + 1; i++) {
      payload[1] = i;
      // Serial.println("Requesting Info for Vehicle:");
      for (j = 0; j < 10; j++) {
        xbee.send(tx1);
        xbee.send(tx2);
        xbee.send(tx3);

      }

      while (go == true) {
        payload[1] = i;
        //Serial.println(payload[1]);
        //Serial.println("do you want the scenario to stop?");
        if (Serial.available()) {
          scenario = Serial.read(); //select the scenario
        }
        if (scenario == '0') {
          cont = false;
          go = false;
        }
        xbee.send(tx1);
        xbee.readPacket(1000);
        //Serial.println("Waiting for Information to come in");

        if (xbee.getResponse().isAvailable()) {
          if (xbee.getResponse().getApiId() == RX_16_RESPONSE) {
            xbee.getResponse().getRx16Response(rx16);
            CarID = rx16.getData(0);
            scenario1 = rx16.getData(1);
            velocity = rx16.getData(2);
            distance = rx16.getData(3);
          }

        }
        if (CarID == i) {
          go = false;
          Serial.println();
          Serial.printf("%d ,%d ,%d ,%d \n", CarID, scenario1, velocity, distance);
        }

      }
      go = true;
    }


  }
}
