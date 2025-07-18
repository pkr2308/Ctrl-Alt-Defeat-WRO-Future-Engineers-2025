#include <Arduino.h>
#include <RF24.h>
#include "pindefs.h"
#include "datastruct.h"
#include "cmdstruct.h"
#include "vector.h"
#include "pipeaddr.h"


RF24 radio(PIN_RF24_CE, PIN_RF24_CSN);

RFCommand command;
RFData data;


void setup(){

  SPI.setSCK(PIN_SPI0_SCK);
  SPI.setMOSI(PIN_SPI0_MOSI);
  SPI.setMISO(PIN_SPI0_MISO);

  Serial.begin();  
  while(!Serial);

  if(!radio.begin()){
    Serial.println("Radio init failed");
  }

  radio.setPALevel(RF24_PA_HIGH);
  radio.setDataRate(RF24_2MBPS);
  radio.setAutoAck(true);
  radio.enableAckPayload();
  radio.setAddressWidth(5);  
  
  radio.openReadingPipe(1, VEHICLE_RX_ADDR);
  radio.startListening();

}

void loop(){

  data.orientation.x = 42;

  uint8_t pipe;

  if(radio.available(&pipe)){

    radio.read(&command, sizeof(command));
   
    if(radio.writeAckPayload(pipe, &data, sizeof(data))){

      Serial.println("successfully sent ack payload");

    }

    Serial.println(command.targetHeading);

  }

}