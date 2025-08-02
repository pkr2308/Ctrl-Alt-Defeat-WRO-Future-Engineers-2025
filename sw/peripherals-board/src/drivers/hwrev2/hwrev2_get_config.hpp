/**
 * @file hwrev2_get_config.hpp
 * @brief Implents function to get configuration struct
 * @author DIY Labs
 * @date 2024-01-01
 */

#pragma once

#include <managers/config.hpp>

VehicleConfig hwrev2_getConfig(){

  VehicleConfig cfg;

  cfg.pinConfig.motorDriverDirA             = 28;
  cfg.pinConfig.motorDriverDirB             = 27;
  cfg.pinConfig.motorDriverPWM              = 26;
  cfg.pinConfig.motorDriverStandby          = 29;
  cfg.pinConfig.steeringServo               = 12;
  cfg.pinConfig.uart1RX                     = 15;
  cfg.pinConfig.uart1TX                     = 14;

  cfg.limitsConfig.maxForwardSpeed          = 500;
  cfg.limitsConfig.maxForwardSpeed          = 500;

  return cfg;
  
}