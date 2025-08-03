#pragma once

#include <config.hpp>
#include <ISensor.hpp>
#include <sensordata.hpp>
#include <TFLI2C.h>

class hw_rev_2_lidar : public ISensor{
public:
  hw_rev_2_lidar(VehicleConfig cfg);
  void init() override;
  SensorData update() override;

private:
  VehicleConfig _config;
  TFLI2C *_lidar;

};