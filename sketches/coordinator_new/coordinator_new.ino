//Teensy Coordinator: This set-up is used to select a scenario and get data
//Update 3/13/18: applied a filter to select which car is to send data and will
//only print when the correct Vehicle sent the data if not it will continue to
//scan until it does
//#include <Printers.h>
#include <XBee.h>

///////////XBee Configuration///////////////
XBee xbee = XBee();
uint8_t payload[] = { 0, 0, 0, 0 }; //Car ID, scenario, velocity,distance, braking
XBeeAddress64 addr1 = XBeeAddress64(0x0013a200, 0x412727c9);//
XBeeAddress64 addr2 = XBeeAddress64(0x0013a200, 0x4167d261);
XBeeAddress64 addr3 = XBeeAddress64(0x0013a200, 0x4106a2b4);//
XBeeAddress64 addr4 = XBeeAddress64(0x0013a200, 0x4127385d);//
XBeeAddress64 addr5 = XBeeAddress64(0x0013a200, 0x412727cf);
Tx64Request tx1 = Tx64Request(addr1, payload, sizeof(payload));
Tx64Request tx2 = Tx64Request(addr2, payload, sizeof(payload));
Tx64Request tx3 = Tx64Request(addr3, payload, sizeof(payload));
Tx64Request tx4 = Tx64Request(addr4, payload, sizeof(payload));
Tx64Request tx5 = Tx64Request(addr4, payload, sizeof(payload));

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
int CarID;
int Ton = 13; //teensy led pin
//int s1 = 19; //scenario 1 LED stop and go waves active
bool cont = true;
bool go = true;
int count = 0;
int noc = 1; //number of cars
   double start_time = 0.00 ;
   double current_time = 0.0;
void setup() {
  Serial1.begin(115200);//configured teensy ports for communication
  Serial.begin(115200);//initializes baud rate for the rest of the ports
  xbee.setSerial(Serial1);
  //pinMode(s1, OUTPUT);
  pinMode(Ton, OUTPUT);
  digitalWrite(Ton,HIGH);

  start_time = millis();
}

void loop() {
 
  //Serial.println("Waiting for Information to come in");
  //int current_time;
//  current_time=
//            current_time =  millis() - start_time;
//          current_time = current_time / 1000;
//  while (cont == true) {
//    int current_time = ((double) millis() - start_time);
//    //Serial.println("Program Running");
//    for (i = 1; i < noc + 1; i++) {
//      payload[1] = i;
//              Serial.println(payload[1]);
//      // Serial.println("Requesting Info for Vehicle:");
//
//      for (j = 0; j < 10; j++) {
//        xbee.send(tx1);
//        xbee.send(tx2);
//        xbee.send(tx3);
//        xbee.send(tx4);
//        xbee.send(tx5);
//
//      }
//       while (go == true) {
//
//        //Serial.println("do you want the scenario to stop?");
//        if (Serial.available()) {
//          scenario = Serial.read(); //select the scenario
//        }
//        if (scenario == '0') {
//          cont = false;
//          go = false;
//        }
//        xbee.send(tx1);
//        xbee.readPacket(1000);
//        //Serial.println("Waiting for Information to come in");
//
//        if (xbee.getResponse().isAvailable()) {
//          if (xbee.getResponse().getApiId() == RX_16_RESPONSE) {
//            xbee.getResponse().getRx16Response(rx16);
//            CarID = rx16.getData(0);
//            scenario1 = rx16.getData(1);
//            velocity = rx16.getData(2);
//            distance = rx16.getData(3);
//          }
//
//        }
//        if (CarID == i) {
//          go = false;
//          count = count + 1;
//          current_time = ((double) millis() - start_time);
//          current_time = current_time / 1000;
//          Serial.println();
//          Serial.printf("%d ,%d ,%d ,%d,%d, %d \n", CarID, scenario1, velocity, distance, count, current_time);
//        }
//      }
//      go=false;
//    }
xbee.send(tx1);
  xbee.readPacket();
  if (xbee.getResponse().isAvailable()) {
    if (xbee.getResponse().getApiId() == RX_16_RESPONSE) {
      xbee.getResponse().getRx16Response(rx16);
      CarID = rx16.getData(0);
      scenario1 = rx16.getData(1);
      velocity = rx16.getData(2);
      distance = rx16.getData(3);
                  current_time =  millis() - start_time;
          current_time = current_time / 1000;
          count=count+1;
      Serial.println();
      Serial.printf("%d ,%d ,%d ,%d,%d, %f \n", CarID, scenario1, velocity, distance, count, current_time);
    }

  }
}

