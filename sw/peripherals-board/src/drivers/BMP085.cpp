/*
  BMP085.cpp
  Driver for BMP075 barometer
  Written by DIY Labs

*/

#define DRIVER_BMP085_DEBUG   // Uncomment to enable debug messages

#include "BMP085.h"
#include <Adafruit_BMP085.h>
#include <Adafruit_Sensor.h>

DriverBMP085::DriverBMP085(TwoWire* wire){

  _wire = wire;
  _bmp = Adafruit_BMP085();
 
}

bool DriverBMP085::initialize(){

  if(!_bmp.begin(BMP085_ULTRAHIGHRES, _wire)){

    #ifdef DRIVER_BMP085_DEBUG
      Serial.println("DriverBMP085: Failed to initialize BMP085");
    #endif
    return false;

  }
  else{

    #ifdef DRIVER_BMP085_DEBUG
      Serial.println("DriverBMP085: Successfully initialized BMP085");
    #endif

    return true;

  }

}

std::vector<sensors_event_t> DriverBMP085::update(){

  std::vector<sensors_event_t> data;
  sensors_event_t tempEvent, presEvent;  

  float temp = _bmp.readTemperature();
  float pressure = _bmp.readPressure();

  tempEvent.version = sizeof(sensors_event_t);
  tempEvent.sensor_id = 0;
  tempEvent.timestamp = millis();
  tempEvent.type = SENSOR_TYPE_AMBIENT_TEMPERATURE;
  tempEvent.temperature = temp;
  
  presEvent.version = sizeof(sensors_event_t);
  presEvent.sensor_id = 0;
  presEvent.timestamp = millis();
  presEvent.type = SENSOR_TYPE_PRESSURE;
  presEvent.pressure = pressure;
  

  data.push_back(tempEvent);
  data.push_back(presEvent);
  return data;

}

