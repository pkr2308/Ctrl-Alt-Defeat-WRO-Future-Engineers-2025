/**
 * @brief Implementation of hwrev2 target control driver
 * @author DIY Labs
 */

#include <hwrev2_target_control.hpp>
#include <Arduino.h>
#include <PID_v1.h>

hw_rev_2_TargetControl::hw_rev_2_TargetControl(VehicleConfig cfg){

  _config = cfg;

}

void hw_rev_2_TargetControl::init(IMotorDriver* motorDriver, ISteeringDriver* steeringDriver, ILogger* logger){

  _logger = logger;

  _motorDriver = motorDriver;
  _steeringDriver = steeringDriver;
  // TODO: implement nullptr check and log

  steeringPID = new PID(&vehicleYaw, &steeringPIDCommand, &adjustedYawTarget, _config.controlConfig.steeringP, _config.controlConfig.steeringI, _config.controlConfig.steeringD, DIRECT);
  steeringPID->SetMode(AUTOMATIC);
  steeringPID->SetOutputLimits(_config.controlConfig.minSteeringPIDCommand, _config.controlConfig.maxSteeringPIDCommand);
  
  _motorDriver->init(_logger);
  _motorDriver->armMotor();
  _steeringDriver->init(_logger);

}

void hw_rev_2_TargetControl::targetControl(VehicleCommand cmd, VehicleData data){

  adjustedYawTarget = cmd.targetYaw;
  vehicleYaw = data.orientation.x;

  yawError = getShortestAngleError(adjustedYawTarget, vehicleYaw);

  if(data.orientation.z - adjustedYawTarget > 180){
    adjustedYawTarget += 360;
  }  
  else if(data.orientation.z - adjustedYawTarget < -180){
    adjustedYawTarget -= 360;
  }
  
  steeringPID->Compute();

  _steeringDriver->steer(map(steeringPIDCommand, _config.controlConfig.minSteeringPIDCommand, _config.controlConfig.maxSteeringPIDCommand, -_config.controlConfig.maxSteeringAngle, _config.controlConfig.maxSteeringAngle));
  _motorDriver->driveMotor(abs(cmd.targetSpeed), cmd.targetSpeed); // temporary, implement PID speed control when reliable speed can be calculated from encoder

}

void hw_rev_2_TargetControl::directControl(VehicleCommand cmd, VehicleData data){

  _steeringDriver->steer(map(cmd.targetYaw, 0, 180, -_config.controlConfig.maxSteeringAngle, _config.controlConfig.maxSteeringAngle));
  _motorDriver->driveMotor(abs(cmd.targetSpeed), cmd.targetSpeed > 0);

}

double hw_rev_2_TargetControl::getShortestAngleError(double target, double current) {

  double error = target - current;

  if(error > 180){
    error -= 360;
  }

  else if(error < -180){
    error += 360;
  }

  return error;

}