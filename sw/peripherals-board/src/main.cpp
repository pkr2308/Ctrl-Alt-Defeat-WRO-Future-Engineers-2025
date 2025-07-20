#include <Arduino.h>
#include <RF24.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_SSD1306.h>
#include "pindefs.h"
#include "datastruct.h"
#include "cmdstruct.h"
#include "vector.h"
#include "pipeaddr.h"


RF24 radio(PIN_RF24_CE, PIN_RF24_CSN);
Adafruit_BNO055 bno = Adafruit_BNO055(0x28);
Adafruit_SSD1306 oled;

RFCommand rfCommand;
RFData rfData;


void setCommPins();
bool initRadio();


void setup(){

  setCommPins();

  Serial.begin();  
  while(!Serial);

  if(initRadio()){
    Serial.println("Radio init failed");
  }

  if(!bno.begin()){
    Serial.println("BNO055 init failed");
  }

}

void loop(){

  sensors_event_t event; 
  bno.getEvent(&event);

  rfData.orientation.x = event.orientation.x;
  rfData.orientation.y = event.orientation.y;
  rfData.orientation.z = event.orientation.z;

  uint8_t pipe;

  if(radio.available(&pipe)){

    radio.read(&rfCommand, sizeof(rfCommand));
   
    if(radio.writeAckPayload(pipe, &rfData, sizeof(rfData))){

      Serial.println("successfully sent ack payload");

    }

  }

}

void setCommPins(){
  
  SPI.setSCK(PIN_SPI0_SCK);
  SPI.setMOSI(PIN_SPI0_MOSI);
  SPI.setMISO(PIN_SPI0_MISO);

  Wire.setSCL(PIN_I2C0_SCL);
  Wire.setSDA(PIN_I2C0_SDA);

}

bool initRadio(){

  if(!radio.begin()){
    return false;
  }

  radio.setPALevel(RF24_PA_HIGH);
  radio.setDataRate(RF24_2MBPS);
  radio.setAutoAck(true);
  radio.enableAckPayload();
  radio.setAddressWidth(5);  
  
  radio.openReadingPipe(1, VEHICLE_RX_ADDR);
  radio.startListening();

}