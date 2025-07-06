/*
  TFLuna.cpp
  Driver for TFLuna LiDAR 
  Written by DIY Labs

*/

#define DRIVER_TFLuna_DEBUG   // Uncomment to enable debug messages

#include "TFLuna.h"
//#include <TFLI2C.h>         // redefine error
#include <Adafruit_Sensor.h>

DriverTFLuna::DriverTFLuna(){

}

bool DriverTFLuna::initialize(){

  return true;

}

std::vector<sensors_event_t> DriverTFLuna::update(){

  std::vector<sensors_event_t> events;

  sensors_event_t lidarEvent;
  int16_t dist;

  _tfluna.getData(dist, TFL_DEF_ADR);

  lidarEvent.version = sizeof(sensors_event_t);
  lidarEvent.sensor_id = 0;
  lidarEvent.timestamp = millis();
  lidarEvent.type = SENSOR_TYPE_PROXIMITY;
  lidarEvent.distance = dist;

  events.push_back(lidarEvent);

  return events;

}

