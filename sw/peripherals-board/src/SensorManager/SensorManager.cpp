/*
  SensorManager.cpp
  Manages sensor data collection and processing

  Written by DIY Labs

*/

// #define SENSOR_MANAGER_DEBUG   // Uncomment to enable debug messages

#include "SensorManager.h"
#include "Sensor.h"
#include <Arduino.h>

SensorManager::SensorManager(){

}

void SensorManager::addSensor(Sensor* sensor){

  if(sensor){

    _sensors.push_back(sensor);

    #ifdef SENSOR_MANAGER_DEBUG
      Serial.println("SensorManager: Added sensor of type: " + String(sensor->getType()));
    #endif

  }
  else{
    #ifdef SENSOR_MANAGER_DEBUG
      Serial.println("SensorManager: Attempted to add a null sensor");
    #endif
  }


}

bool SensorManager::initializeSensors(){

  bool allInitialized = true;

  for(Sensor* sensor : _sensors){

    if(sensor){

      bool init = sensor->initialize();

      if(init){

        #ifdef SENSOR_MANAGER_DEBUG
          Serial.println("SensorManager: Successfully initialized sensor of type: " + String(sensor->getType()));
        #endif

      }
      else{

        #ifdef SENSOR_MANAGER_DEBUG
          Serial.println("SensorManager: Failed to initialize sensor of type: " + String(sensor->getType()));
        #endif

        allInitialized = false;

      }

    }
    else{

      #ifdef SENSOR_MANAGER_DEBUG
        Serial.println("SensorManager: Null sensor encountered during initialization");
      #endif

      allInitialized = false;

    }

  }

  return allInitialized;

}

std::vector<sensors_event_t> SensorManager::updateSensors(){

  std::vector<sensors_event_t> data;

  for(Sensor* sensor : _sensors){

    if(sensor){

      std::vector<sensors_event_t> sensorData = sensor->update();

      if(!sensorData.empty()){

        #ifdef SENSOR_MANAGER_DEBUG
          Serial.println("SensorManager: Updated sensor of type: " + String(sensor->getType()));
        #endif

        data.insert(data.end(), sensorData.begin(), sensorData.end());

      }
      else{

        #ifdef SENSOR_MANAGER_DEBUG
          Serial.println("SensorManager: No data returned from sensor of type: " + String(sensor->getType()));
        #endif

      }

    }
    else{

      #ifdef SENSOR_MANAGER_DEBUG
        Serial.println("SensorManager: Attempted update on null sensor");
      #endif

      continue;

    }

  }

  return data;

}