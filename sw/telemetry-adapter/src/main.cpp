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
unsigned long commandSendIntervalMS = 100;

RFCommand command;
RFData data;


void blinkLED();


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

  Serial.begin();  
  while(!Serial){
    blinkLED();
  }

  radio.setPALevel(RF24_PA_HIGH);
  radio.setDataRate(RF24_2MBPS);
  radio.setAutoAck(true);
  radio.enableAckPayload();
  radio.setAddressWidth(5);
  
}

void loop(){

  if(millis() - prevCommandSendTime >= commandSendIntervalMS){

    command.targetHeading = 20;
    command.targetSpeed = 200;

    radio.stopListening();
    radio.openWritingPipe(VEHICLE_RX_ADDR);

    bool sent = radio.write(&command, sizeof(command));

    if(sent){

      Serial.println("Successfully sent command, awaiting telemetry");

      if(radio.available()){

        Serial.println("Received telemetry");
        
        radio.read(&data, sizeof(data));

        Serial.print("Orientation X: ");
        Serial.println(data.orientation.x);

      }

    }

  }

}

void blinkLED(){

  pixel.setPixelColor(0, pixel.Color(255, 255, 0));
  pixel.show();
  delay(100);
  pixel.setPixelColor(0, pixel.Color(0, 0, 0));
  pixel.show();
  delay(100);

}