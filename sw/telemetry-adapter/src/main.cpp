#include <Arduino.h>
#include <RF24.h>
#include <Adafruit_NeoPixel.h>
#include "pindefs.h"
#include "hwrev2_rf24_shared.hpp"


RF24 radio(RF_CE, RF_CSN);
Adafruit_NeoPixel pixel(1, PIN_RGB_LED, NEO_RGB + NEO_KHZ800);


unsigned long prevCommandSendTime = 0;
unsigned long commandSendIntervalMS = 10;

hwrev2_rf24_telem_block1 dataBlock1;


void blinkLED();
void printDataStruct();
void ledOn();
void ledOff();


void setup(){

  pixel.begin();
  pixel.setBrightness(5);

  SPI.setSCK(RF_SCK);
  SPI.setMOSI(RF_MOSI);
  SPI.setMISO(RF_MISO);

  if(!radio.begin()){
    pixel.setPixelColor(0, pixel.Color(255, 0, 0));
    pixel.show();
    while(true);
  }
  pixel.setPixelColor(0, pixel.Color(0, 255, 0));
  pixel.show();

  Serial.begin();  

  radio.setPALevel(RF24_PA_HIGH);
  radio.setDataRate(RF24_2MBPS);
  radio.setAutoAck(true);
  radio.enableAckPayload();
  radio.setAddressWidth(5);
  
}

void loop(){

  radio.openReadingPipe(1, TLM_PIPE_0);
  radio.startListening();

  if(radio.available()){

    radio.read(&dataBlock1, sizeof(dataBlock1));

    printDataStruct();

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

  Serial.print(dataBlock1.oriX);
  Serial.print(',');

  Serial.print(dataBlock1.oriY);
  Serial.print(',');

  Serial.print(dataBlock1.oriZ);
  Serial.print(',');

  Serial.print(dataBlock1.lidarLeft);
  Serial.print(',');  

  Serial.print(dataBlock1.lidarFront);
  Serial.print(',');  

  Serial.print(dataBlock1.lidarRight);
  Serial.print(',');  

  Serial.print(dataBlock1.commandedSpeed);
  Serial.print(',');  

  Serial.print(dataBlock1.commandedSteer);
  Serial.print(',');  

  Serial.print(dataBlock1.imuCalib);
  Serial.print(',');  

  Serial.print(dataBlock1.gyroCalib);
  Serial.print(',');  

  Serial.print(dataBlock1.accelCalib);
  Serial.print(',');  

  Serial.print(dataBlock1.magCalib);
  Serial.print(',');  

  Serial.print(dataBlock1.speed);
  Serial.print(',');  

  Serial.println();

}