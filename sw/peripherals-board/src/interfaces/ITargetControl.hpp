#pragma once
#include <IMotorDriver.hpp>
#include <ISteeringDriver.hpp>
#include <vehiclecommand.hpp>
#include <vehicledata.hpp>  

class ITargetControl{
public:  

  virtual ~ITargetControl() = default;

  virtual void init(IMotorDriver* motorDriver, ISteeringDriver* steeringDriver) = 0;
  virtual void targetControl(VehicleCommand cmd, VehicleData data) = 0;
  virtual void directControl(VehicleCommand cmd, VehicleData data) = 0; 
  
};