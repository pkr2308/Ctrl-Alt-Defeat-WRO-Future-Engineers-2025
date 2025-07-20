#include <Arduino.h>
#include <RF24.h>
#include <Adafruit_BNO055.h>
#include "pindefs.h"
#include "datastruct.h"
#include "cmdstruct.h"
#include "vector.h"
#include "pipeaddr.h"


RF24 radio(PIN_RF24_CE, PIN_RF24_CSN);
Adafruit_BNO055 bno = Adafruit_BNO055(0x28);

RFCommand command;
RFData data;


void setup(){

  SPI.setSCK(PIN_SPI0_SCK);
  SPI.setMOSI(PIN_SPI0_MOSI);
  SPI.setMISO(PIN_SPI0_MISO);

  Wire.setSCL(PIN_I2C0_SCL);
  Wire.setSDA(PIN_I2C0_SDA);

  Serial.begin();  
  while(!Serial);

  if(!radio.begin()){
    Serial.println("Radio init failed");
  }

  if(!bno.begin()){
    Serial.println("BNO055 init failed");
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

  sensors_event_t event; 
  uint8_t sysCal, accelCal, gyroCal, magCal;
  bno.getCalibration(&sysCal, &gyroCal, &accelCal, &magCal);
  bno.getEvent(&event);

  data.orientation.x = event.orientation.x;
  data.orientation.y = event.orientation.y;
  data.orientation.z = event.orientation.z;
  data.bnoSysCal = sysCal;
  data.bnoGyroCal = gyroCal;
  data.bnoAccelCal = accelCal;
  data.bnoMagCal = magCal;
  data.heartbeat = !data.heartbeat;

  uint8_t pipe;

  if(radio.available(&pipe)){

    radio.read(&command, sizeof(command));
   
    if(radio.writeAckPayload(pipe, &data, sizeof(data))){

      Serial.println("successfully sent ack payload");

    }

    Serial.println(command.targetHeading);

  }

}