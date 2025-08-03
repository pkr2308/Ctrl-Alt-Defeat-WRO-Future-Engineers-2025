#pragma once

#include <config.hpp>
#include <vehiclecommand.hpp>
#include <vehicledata.hpp>
#include <IMotorDriver.hpp>
#include <ISteeringDriver.hpp>
#include <ITargetControl.hpp>
#include <PID_v1.h>

class hw_rev_2_TargetControl : public ITargetControl{
public:
  hw_rev_2_TargetControl
(VehicleConfig cfg);
  void init(IMotorDriver* motorDriver, ISteeringDriver* steeringDriver) override;
  void targetControl(VehicleCommand cmd, VehicleData data) override;
  void directControl(VehicleCommand cmd, VehicleData data) override;

private:
  VehicleConfig _config;
  IMotorDriver* _motorDriver;
  ISteeringDriver* _steeringDriver;

  PID *steeringPID;
  PID *speedPID;

  double vehicleYaw;
  double yawError;
  double yawTarget;
  double adjustedYawTarget;
  double steeringPIDCommand;

  double getShortestAngleError(double target, double current);

};