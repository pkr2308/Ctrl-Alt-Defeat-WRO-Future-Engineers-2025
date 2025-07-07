/*
  DataProcessor.cpp
  Utility class to processes data from IMU into vehicle state

  Written by DIY Labs

*/

#include "DataProcessor.h"
#include "VehicleState.h"
#include <vector>
#include <Adafruit_Sensor.h>

#define DATA_PROCESSOR_DEBUG 

DataProcessor::DataProcessor(){}

VehicleState DataProcessor::processSensorData(std::vector<sensors_event_t> sensorData) {
  
  _currentMillis = millis();

  for(sensors_event_t event : sensorData){

    if(event.type == SENSOR_TYPE_GYROSCOPE){

      _state.rawGyro.x = event.gyro.x;
      _state.rawGyro.y = event.gyro.y;
      _state.rawGyro.z = event.gyro.z;

      float dt = (_currentMillis - _prevMillis) / 1000.0f; // convert ms to seconds

      if(event.gyro.x >= 0.1 || event.gyro.x <= -0.1) _state.orientation.x += event.gyro.x * (180.0f / PI) * dt;
      if(event.gyro.y >= 0.1 || event.gyro.y <= -0.1) _state.orientation.y += event.gyro.y * (180.0f / PI) * dt;
      if(event.gyro.z >= 0.1 || event.gyro.z <= -0.1) _state.orientation.z += event.gyro.z * (180.0f / PI) * dt;
      

      break; // stops at first gyro, assume we only have one gyro/gyro event

    }

  }

  _prevMillis = _currentMillis;
  return _state;

}

