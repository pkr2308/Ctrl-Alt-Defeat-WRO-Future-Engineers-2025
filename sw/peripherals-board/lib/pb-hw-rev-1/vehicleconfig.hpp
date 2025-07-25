/**
 * @brief Sets up a configuration struct for the Waveshare RP2040-zero based peripherals board
 * @author DIY Labs
 */

#pragma once

#include <configuration.hpp>


VehicleConfig getVehicleConfig(){

  VehicleConfig cfg;

  cfg.pinConfig.motorDriverPWM = 26;
  cfg.pinConfig.motorDriverDirA = 28;
  cfg.pinConfig.motorDriverDirB = 27;
  cfg.pinConfig.motorDriverStandby = 29;

  cfg.pinConfig.steeringServo = 12;
  

  return cfg;

}