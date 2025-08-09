/*
#include "hwrev1_steering_driver.hpp"
#include <interfaces/ISteeringDriver.hpp>
#include <managers/config.hpp>
#include <Arduino.h>
#include <Servo.h>

hw_rev_1_SteeringDriver::hw_rev_1_SteeringDriver(VehicleConfig cfg){

  _config = cfg;

}

void hw_rev_1_SteeringDriver::init(){

  _steeringServo.attach(_config.pinConfig.steeringServo);
  _steeringServo.write(_SERVO_CENTER);

}

void hw_rev_1_SteeringDriver::steer(int steeringAngle){

  int servoCommand = map((steeringAngle * _ratio), _COMMAND_MIN, _COMMAND_MAX, _SERVO_MIN, _SERVO_MAX);
  _steeringServo.write(servoCommand);

}

*/