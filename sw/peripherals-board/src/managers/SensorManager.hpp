/**
 * @brief Class for updating sensors and transforming sensor data into VehicleData
 * @author DIY Labs
 */

#pragma once

#include "vec3f.hpp"
#include <vector>
#include <config.hpp>
#include <ISensor.hpp>
#include <ILogger.hpp>
#include <vehicledata.hpp>
#include <sensordata.hpp>

class SensorManager{

public:
  SensorManager(VehicleConfig cfg);
  bool addSensor(ISensor* sensor);
  void init(ILogger *logger);
  VehicleData update();

private:
  VehicleConfig _config;
  std::vector<ISensor*> _sensors;
  ILogger *_logger;

};