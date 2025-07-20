#include <Arduino.h>
#include <RF24.h>
#include <Adafruit_NeoPixel.h>
#include "pindefs.h"
#include "datastruct.h"
#include "cmdstruct.h"
#include "vector.h"
#include "pipeaddr.h"


RF24 radio(RF_CE, RF_CSN);
Adafruit_NeoPixel pixel(1, PIN_RGB_LED, NEO_RGB + NEO_KHZ800);


unsigned long prevCommandSendTime = 0;
unsigned long commandSendIntervalMS = 10;

RFCommand command;
RFData data;


void blinkLED();
void printDataStruct();
void ledOn();
void ledOff();
bool initRadio();
void getCommandsFromSerial();
void handleRadio();


void setup(){

  pixel.begin();
  pixel.setBrightness(5);

  SPI.setSCK(RF_SCK);
  SPI.setMOSI(RF_MOSI);
  SPI.setMISO(RF_MISO);

  Serial.begin();
  
}

void loop(){

  if(millis() - prevCommandSendTime >= commandSendIntervalMS){

    prevCommandSendTime = millis();

    getCommandsFromSerial();
    handleRadio();

  }

}

void ledOn(){

  pixel.setPixelColor(0, pixel.Color(255, 255, 0));
  pixel.show();

}

void ledOff(){

  pixel.setPixelColor(0, pixel.Color(0, 0, 0));
  pixel.show();
  
}

void printDataStruct(){

  Serial.print(data.orientation.x);
  Serial.print(',');

  Serial.print(data.orientation.y);
  Serial.print(',');

  Serial.print(data.orientation.z);
  Serial.print(',');

  Serial.print(data.speed);
  Serial.print(',');  

  Serial.print(data.steeringAngle);
  Serial.print(',');  

  Serial.print(data.targetSpeed);
  Serial.print(',');  

  Serial.print(data.targetYaw);
  Serial.print(',');  

  Serial.print(data.lidarData[0]);
  Serial.print(',');  

  Serial.print(data.lidarData[1]);
  Serial.print(',');  

  Serial.print(data.lidarData[2]);
  Serial.print(',');  

  Serial.print(data.bnoSysCal);
  Serial.print(',');

  Serial.print(data.bnoGyroCal);
  Serial.print(',');

  Serial.print(data.bnoAccelCal);
  Serial.print(',');

  Serial.print(data.bnoMagCal);
  Serial.print(',');

  Serial.println();

}

bool initRadio(){

  if(!radio.begin()){

    pixel.setPixelColor(0, pixel.Color(255, 0, 0));
    pixel.show();
    return false;

  }

  radio.setPALevel(RF24_PA_HIGH);
  radio.setDataRate(RF24_2MBPS);
  radio.setAutoAck(true);
  radio.enableAckPayload();
  radio.setAddressWidth(5);
  return true;

}

void handleRadio(){

  radio.stopListening();
  radio.openWritingPipe(VEHICLE_RX_ADDR);

  bool sent = radio.write(&command, sizeof(command));

  if(sent){

    ledOff();

    if(radio.available()){
      
      radio.read(&data, sizeof(data));

      printDataStruct();

    }

  }
  else{
    ledOn();
  }

}