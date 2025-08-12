/*
#include <hwrev1_target_control.hpp>

hw_rev_1_TargetControl::hw_rev_1_TargetControl(VehicleConfig cfg){

  _config =cfg;

}

void hw_rev_1_TargetControl::init(IMotorDriver* motorDriver, ISteeringDriver* steeringDriver){

  _motorDriver = motorDriver;
  _steeringDriver = steeringDriver;

  // TODO: implement nullptr check and log

  _motorDriver->init();
  _motorDriver->armMotor();
  _steeringDriver->init();

}

void hw_rev_1_TargetControl::targetControl(VehicleCommand cmd){

  // implement PID

  _steeringDriver->steer(cmd.targetYaw);
  _motorDriver->driveMotor(cmd.targetSpeed, true);
  _motorDriver->driveMotor(cmd.targetSpeed, true);

}
*/