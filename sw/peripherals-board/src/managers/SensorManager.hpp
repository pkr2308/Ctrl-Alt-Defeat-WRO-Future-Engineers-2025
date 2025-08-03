#pragma once

#include "vec3f.hpp"
#include <vector>
#include <config.hpp>
#include <ISensor.hpp>
#include <vehicledata.hpp>
#include <sensordata.hpp>

class SensorManager{

public:
  SensorManager(VehicleConfig cfg);
  void addSensor(ISensor* sensor);
  void init();
  VehicleData update();

private:
  VehicleConfig _config;
  std::vector<ISensor*> _sensors;

};