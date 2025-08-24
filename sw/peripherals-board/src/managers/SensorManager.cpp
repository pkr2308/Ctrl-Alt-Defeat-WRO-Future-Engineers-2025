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
#include <status.hpp>
#include <Arduino.h>

SensorManager::SensorManager(VehicleConfig cfg){
  _config = cfg;
}

bool SensorManager::addSensor(ISensor* sensor){

  status_t status = sensor->init(_logger);

  if(status == STATUS_HEALTHY){
  
    _logger->sendMessage("SensorManager::addSensor()", _logger->INFO, "Sensor " + sensor->getSensorName()+ " initialized successfully");
    _sensors.push_back(sensor); 
    _logger->sendMessage("SensorManager::addSensor()", _logger->INFO, "Added " + sensor->getSensorName()+ " to SensorManager");
    return true;
  
  }

  else if(status == STATUS_FAULT){
    _logger->sendMessage("SensorManager::addSensor()", _logger->INFO, "Sensor " + sensor->getSensorName()+ " failed to initialize and was not added to SensorManager");
    return false;
  }

  return false; // We should have returned before this, if we get to this point, something's definitely wrong

}

void SensorManager::init(ILogger *logger){

  _logger = logger;
  _logger->sendMessage("SensorManager::init()", _logger->INFO, "Finished initialising sensor manager");

}

VehicleData SensorManager::update(){

  std::vector<std::vector<SensorData>> sensorDataVector;
  VehicleData vehicleData;

  for(ISensor* sensor : _sensors){

    sensorDataVector.push_back(sensor->update()); // TODO: nullptr and failure check
    _logger->sendMessage("SensorManager::update()", _logger->INFO, "Got data from " + sensor->getSensorName());

  }

  for(std::vector<SensorData> dataVector : sensorDataVector){

    for(SensorData data : dataVector){

      if(data.sensorDataType == SENSOR_IMU){
        _logger->sendMessage("SensorManager::update()", _logger->INFO, "Parsing data of type IMU");
        vehicleData.orientation = data.orientation;
        vehicleData.angularVelocity = data.angularVelocity;
        vehicleData.acceleration = data.acceleration;
      }

      else if(data.sensorDataType == SENSOR_LIDAR){
        _logger->sendMessage("SensorManager::update()", _logger->INFO, "Parsing data of type LIDAR");
        
        for(int i = 0; i < 360; i++){
          vehicleData.lidar[i] = data.lidar[i];
        }

      }

      else if (data.sensorDataType == SENSOR_ENCODER){
        _logger->sendMessage("SensorManager::update()", _logger->INFO, "Parsing data of type ENCODER");      
        vehicleData.encoderPosition = data.encoderPosition;
      }

      else if (data.sensorDataType == SENSOR_SPEED){
        _logger->sendMessage("SensorManager::update()", _logger->INFO, "Parsing data of type SPEED");            
        vehicleData.speed = data.speed;
      }

      else if (data.sensorDataType == SENSOR_IMU_CALIBRATION){
        _logger->sendMessage("SensorManager::update()", _logger->INFO, "Parsing data of type IMU_CALIBRATION");            
        vehicleData.imuCalib = data.sysCalib;
        vehicleData.gyroCalib = data.gyroCalib;
        vehicleData.accelCalib = data.accelCalib;
        vehicleData.magCalib = data.magCalib;
      }

    }

  }

  return vehicleData;

}