#include <hwrev2_target_control.hpp>

hw_rev_2_TargetControl::hw_rev_2_TargetControl(VehicleConfig cfg){

  _config =cfg;

}

void hw_rev_2_TargetControl::init(IMotorDriver* motorDriver, ISteeringDriver* steeringDriver){

  _motorDriver = motorDriver;
  _steeringDriver = steeringDriver;

  // TODO: implement nullptr check and log

  _motorDriver->init();
  _motorDriver->armMotor();
  _steeringDriver->init();

}

void hw_rev_2_TargetControl::targetControl(VehicleCommand cmd){

  // implement PID

  _steeringDriver->steer(cmd.targetYaw);
  _motorDriver->driveMotor(cmd.targetSpeed, true);
  _motorDriver->driveMotor(cmd.targetSpeed, true);

}