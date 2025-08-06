/**
 * @brief Implementation of SensorManager class
 * @author DIY Labs
 */

#include "vec3f.hpp"
#include <vector>
#include <config.hpp>
#include <ISensor.hpp>
#include <vehicledata.hpp>
#include <sensordata.hpp>
#include "SensorManager.hpp"
#include <Arduino.h>

SensorManager::SensorManager(VehicleConfig cfg){
  _config = cfg;
}

void SensorManager::addSensor(ISensor* sensor){
  _sensors.push_back(sensor); //  todo: implement nullptr and invalid sensor check
}

void SensorManager::init(){

  for(ISensor* sensor : _sensors){

    sensor->init();

  }

}

VehicleData SensorManager::update(){

  std::vector<std::vector<SensorData>> sensorDataVector;
  VehicleData vehicleData;

  for(ISensor* sensor : _sensors){

    sensorDataVector.push_back(sensor->update()); // TODO: nullptr and failure check

  }

  for(std::vector<SensorData> dataVector : sensorDataVector){

    for(SensorData data : dataVector){

      if(data.sensorDataType == SENSOR_IMU){
        vehicleData.orientation = data.orientation;
        vehicleData.angularVelocity = data.angularVelocity;
        vehicleData.acceleration = data.acceleration;
      }

      else if(data.sensorDataType == SENSOR_LIDAR){
        
        for(int i = 0; i < 360; i++){
          vehicleData.lidar[i] = data.lidar[i];
        }

      }

      else if (data.sensorDataType == SENSOR_ENCODER){
        vehicleData.encoderPosition = data.encoderPosition;
      }

    }

  }

  return vehicleData;

}