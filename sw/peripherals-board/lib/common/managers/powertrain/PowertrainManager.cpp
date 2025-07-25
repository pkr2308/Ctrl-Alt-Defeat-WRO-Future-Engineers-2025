/**
 * @file PowertrainManager.cpp
 * @brief Handles drive motor and steering servo
 * @author DIY Labs
 */

 #pragma once

 #include "PowertrainManager.hpp"
 #include <cstdint>

PowertrainManager::PowertrainManager(PowertrainDriver& ptd, VehicleConfig& cfg){

  _driver = ptd;
  _config = cfg;

}

void PowertrainManager::init(){

  _driver.init(_config);

}

void PowertrainManager::drive(uint16_t speed, uint8_t steeringAngle){

  // implement limit logic here

  _driver.commandMotor(speed, (speed > 0), _absMaxMotorCommand);
  _driver.commandSteering(steeringAngle);

}