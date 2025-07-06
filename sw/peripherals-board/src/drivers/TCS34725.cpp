/*
  TCS34725.cpp
  Driver for TCS34725 color sensor
  Written by DIY Labs

*/

#define DRIVER_TCS34725_DEBUG   // Uncomment to enable debug messages

#include "TCS34725.h"
#include <Adafruit_TCS34725.h>
#include <Adafruit_Sensor.h>

DriverTCS34725::DriverTCS34725(){
 
}

bool DriverTCS34725::initialize(){

  if(!_tcs.begin()){

    #ifdef DRIVER_TCS34725_DEBUG
      Serial.println("DriverTCS34725: Failed to initialize TCS34725");
    #endif
    return false;

  }
  else{

    #ifdef DRIVER_TCS34725_DEBUG
      Serial.println("DriverTCS34725: Successfully initialized TCS34725");
    #endif

    return true;

  }

}

std::vector<sensors_event_t> DriverTCS34725::update(){

  std::vector<sensors_event_t> events;
  sensors_event_t eventColor;
  sensors_color_t colorData;

  float r, g, b;
  _tcs.getRGB(&r, &g, &b);
  colorData.r = r;
  colorData.g = g;
  colorData.b = b;

  eventColor.version = sizeof(sensors_event_t);
  eventColor.sensor_id = 0;
  eventColor.timestamp = millis();
  eventColor.type = SENSOR_TYPE_COLOR;
  eventColor.color = colorData; 

  events.push_back(eventColor);

  return events;

}

