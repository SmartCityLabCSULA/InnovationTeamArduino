#include <Printers.h>
#include <XBee.h>
///////////////////////////XBee COnfiguration Start //////////////////////
XBee xbee = XBee();
uint8_t payload[] = {0, 0, 0};
XBeeAddress64 addr64 = XBeeAddress64(0x0013a200, 0x4167d263); // Address of HUB
Tx64Request tx = Tx64Request(addr64, payload, sizeof(payload));
XBeeResponse response = XBeeResponse();
Rx16Response rx16 = Rx16Response();
Rx64Response rx64 = Rx64Response();
///////////////////////////XBee COnfiguration End//////////////////////
uint8_t scenario;
uint8_t timeLeft;
uint8_t Distance;
boolean start = true;



void setup() {
  Serial.begin(57600);
  xbee.setSerial(Serial);

}

void loop() {

  xbee.readPacket();
  Serial.print("Program Started");
  Serial.print('\n');
  if (xbee.getResponse().isAvailable()) {
    Serial.print("in the if loop");
    Serial.print('\n');
    if ( xbee.getResponse().getApiId() == RX_16_RESPONSE) {
      xbee.getResponse().getRx16Response(rx16);
      scenario = rx16.getData(0);
    }
    if ( xbee.getResponse().getApiId() == RX_64_RESPONSE) {
      xbee.getResponse().getRx16Response(rx64);
      scenario = rx64.getData(0);
    }
  }
  Serial.print("Scenario");
  Serial.print('\t');
  Serial.print(scenario);
  Serial.print('\n');
  switch (scenario) {
    case 1://Stop and Go Scenario
      Serial.print('\n');
      Serial.print("Stop and Go Wave Scenario Selected");
      Serial.print('\n');
      StopGoWave();
      scenario=0;
      break;
    case 2: //Eco Stop Scenario
      Serial.print('\n');
      Serial.print("Eco Stop Scenario Selected");
      Serial.print('\n');
      EcoStop();
            scenario=0;
      break;
    case 3:
      Serial.print('\n');
      Serial.print("Stop Sign Scenario Selected");
      Serial.print('\n');
      StopSign();
            scenario=0;
      break;
    case 4:
      Serial.print('\n');
      Serial.print("Stop");
      Serial.print('\n');
      MotorStop();
            scenario=0;
      break;
      default:
            scenario=0;
            break;
  }
}
////////////////////Stop Go Wave Function Start//////////////////////////
void StopGoWave() {
  Serial.print("Stop Go Data Collection Beginning");
  Serial.print('\n');
  while (start == true) {
    //////Speed Algorithm payload[2]//////////
    payload[0] = 3 & 0xff; // vehicle ID
    payload[1] = 4 & 0xff; // secenario 1
    payload[2] = 255 & 0xff; // this should be speed
    xbee.send(tx);

    xbee.readPacket();
    if (xbee.getResponse().isAvailable()) {
      if ( xbee.getResponse().getApiId() == RX_16_RESPONSE) {
        xbee.getResponse().getRx16Response(rx16);
        scenario = rx16.getData(0);
        if (scenario == 9) {
          start = false;
        }
      }
      if ( xbee.getResponse().getApiId() == RX_64_RESPONSE) {
        xbee.getResponse().getRx16Response(rx64);
        scenario = rx64.getData(0);
        if (scenario == 9) {
          start = false;
        }
      }
    }

  }
}
///////////////////Stop Go Wave FUnction End////////////////////////////

//////////////////Eco Stop Function Start///////////////////////////////
void EcoStop() {
  xbee.readPacket();
  Serial.print("Program Started");
  Serial.print('\n');
  if (xbee.getResponse().isAvailable()) {
    Serial.print("in the if loop");
    Serial.print('\n');
    if ( xbee.getResponse().getApiId() == RX_16_RESPONSE) {
      xbee.getResponse().getRx16Response(rx16);
      scenario = rx16.getData(0);
      timeLeft = rx16.getData(1);
      Distance = rx16.getData(2);
    }
    if ( xbee.getResponse().getApiId() == RX_64_RESPONSE) {
      xbee.getResponse().getRx16Response(rx64);
      scenario = rx64.getData(0);
      timeLeft = rx64.getData(1);
      Distance = rx64.getData(2);
    }
  }
  ////////////Motor Algorithm//////////////
}
//////////////////Eco Stop Function End///////////////////////////////

/////////////////Stop Sign Function Start////////////////////////////////
void StopSign() {
  xbee.readPacket();
  Serial.print("Program Started");
  Serial.print('\n');
  if (xbee.getResponse().isAvailable()) {
    Serial.print("in the if loop");
    Serial.print('\n');
    if ( xbee.getResponse().getApiId() == RX_16_RESPONSE) {
      xbee.getResponse().getRx16Response(rx16);
      Distance = rx16.getData(2);
    }
    if ( xbee.getResponse().getApiId() == RX_64_RESPONSE) {
      xbee.getResponse().getRx16Response(rx64);
      Distance = rx64.getData(2);
    }
  }
  ////////////Motor Algorithm//////////////
}

/////////////////Stop Sign Function End////////////////////////////////

///////////////Discontinue Motor Start////////////////////////////////
void MotorStop(){
    xbee.readPacket();
  Serial.print("Program Started");
  Serial.print('\n');
  if (xbee.getResponse().isAvailable()) {
    Serial.print("in the if loop");
    Serial.print('\n');
    if ( xbee.getResponse().getApiId() == RX_16_RESPONSE) {
      xbee.getResponse().getRx16Response(rx16);
      scenario = rx16.getData(0);
    }
    if ( xbee.getResponse().getApiId() == RX_64_RESPONSE) {
      xbee.getResponse().getRx16Response(rx64);
      scenario = rx64.getData(0);
    }
  }

}

///////////////Discontinue Motor End////////////////////////////////

///////////////////////Generic Data Start/////////////////////////////
/*
   Serial.print('\n');
  xbee.readPacket();
  Serial.print("Program Started");
  Serial.print('\n');
  if (xbee.getResponse().isAvailable()) {
    Serial.print("in the if loop");
    Serial.print('\n');
    if ( xbee.getResponse().getApiId() == RX_16_RESPONSE) {
      xbee.getResponse().getRx16Response(rx16);
      scenario = rx16.getData(0);
    }
    if ( xbee.getResponse().getApiId() == RX_64_RESPONSE) {
      xbee.getResponse().getRx16Response(rx64);
      scenario = rx64.getData(0);
    }
  }
  Serial.print("Scenario");
  Serial.print('\t');
  Serial.print(scenario);
  Serial.print('\n');
  delay(1000);
  Serial.flush();




*/
///////////////////////////////Generic Data Read End////////////////////////
