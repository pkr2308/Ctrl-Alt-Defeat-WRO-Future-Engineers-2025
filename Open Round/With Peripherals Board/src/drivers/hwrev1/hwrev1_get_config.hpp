#include <managers/config.hpp>

VehicleConfig hwrev1_getConfig(){

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