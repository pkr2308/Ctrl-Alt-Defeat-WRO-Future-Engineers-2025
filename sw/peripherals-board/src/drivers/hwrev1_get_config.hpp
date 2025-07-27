#include <managers/config.hpp>

VehicleConfig hwrev1_getConfig(){

  VehicleConfig cfg;

  cfg.pinConfig.motorDriverDirA             = 28;
  cfg.pinConfig.motorDriverDirB             = 27;
  cfg.pinConfig.motorDriverPWM              = 26;
  cfg.pinConfig.motorDriverStandby          = 29;
  cfg.pinConfig.steeringServo               = 12;

  return cfg;
  
}