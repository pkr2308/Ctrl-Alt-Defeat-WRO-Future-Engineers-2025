#include "vec3f.hpp"
#include <vector>
#include <config.hpp>
#include <ISensor.hpp>
#include <vehicledata.hpp>
#include <sensordata.hpp>
#include "SensorManager.hpp"

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

  std::vector<SensorData> sensorData;
  VehicleData vehicleData;

  for(ISensor* sensor : _sensors){

    sensorData.push_back(sensor->update()); // TODO: nullptr and failure check

  }

  for(SensorData data : sensorData){

    if(data.sensorDataType == SENSOR_ORIENTATION){
      vehicleData.orientation = data.orientation;
    }

    else if(data.sensorDataType == SENSOR_LIDAR){
      
      for(int i = 0; i < 360; i++){
        vehicleData.lidar[i] = data.lidar[i];
      }

    }

  }

  return vehicleData;

}