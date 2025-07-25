/**
 * @file PowertrainManager.hpp
 * @brief Handles drive motor and steering servo
 * @author DIY Labs
 */

 #pragma once

#include "driver.hpp"
#include <configuration.hpp>
#include <cstdint>

class PowertrainManager{

public:
  PowertrainManager(VehicleConfig cfg);
  ~PowertrainManager() = default;

  void init();
  void drive(uint16_t speed, uint8_t steeringAngle);


private:
  PowertrainDriver _driver;
  VehicleConfig _config;
  uint16_t _absMaxMotorCommand = 1024;

};