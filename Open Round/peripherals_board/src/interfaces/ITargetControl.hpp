/**
 * @brief Interface for target control system
 * @author DIY Labs
 */

#pragma once
#include <IMotorDriver.hpp>
#include <ISteeringDriver.hpp>
#include <vehiclecommand.hpp>
#include <vehicledata.hpp>  
#include <ILogger.hpp>

class ITargetControl{
public:  

  virtual ~ITargetControl() = default;

  virtual void init(IMotorDriver* motorDriver, ISteeringDriver* steeringDriver, ILogger* logger) = 0;
  virtual void targetControl(VehicleCommand cmd, VehicleData data) = 0;
  virtual void directControl(VehicleCommand cmd, VehicleData data) = 0; 
  
};