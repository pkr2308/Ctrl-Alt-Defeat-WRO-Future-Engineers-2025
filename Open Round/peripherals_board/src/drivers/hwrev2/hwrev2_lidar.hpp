/**
 * @brief Header for hwrev2 lidar driver
 * @author DIY Labs
 */

#pragma once

#include <config.hpp>
#include <ISensor.hpp>
#include <sensordata.hpp>
#include <TFLI2C.h>

class hw_rev_2_lidar : public ISensor{
public:
  hw_rev_2_lidar(VehicleConfig cfg);
  void init(ILogger *logger) override;
  std::vector<SensorData> update() override;

private:
  VehicleConfig _config;
  TFLI2C *_lidar;
  ILogger *_logger;

};