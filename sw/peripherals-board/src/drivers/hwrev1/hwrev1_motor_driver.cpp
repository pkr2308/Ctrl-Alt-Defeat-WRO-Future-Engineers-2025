/*
#include "hwrev1_motor_driver.hpp"
#include <interfaces/IMotorDriver.hpp>
#include <managers/config.hpp>
#include <Arduino.h>

hw_rev_1_MotorDriver::hw_rev_1_MotorDriver(VehicleConfig cfg){

  _config = cfg;

}

void hw_rev_1_MotorDriver::init(){

  pinMode(_config.pinConfig.motorDriverPWM, OUTPUT);
  pinMode(_config.pinConfig.motorDriverDirA, OUTPUT);
  pinMode(_config.pinConfig.motorDriverDirB, OUTPUT);
  pinMode(_config.pinConfig.motorDriverStandby, OUTPUT);

  disarmMotor();

  analogWriteResolution(absMaxMotorCommand);

}

void hw_rev_1_MotorDriver::disarmMotor(){

  digitalWrite(_config.pinConfig.motorDriverStandby, LOW);

}

void hw_rev_1_MotorDriver::armMotor(){

  digitalWrite(_config.pinConfig.motorDriverStandby, HIGH);

}

void hw_rev_1_MotorDriver::driveMotor(int speed, bool dir){

  speed = constrain(abs(speed), 0, absMaxMotorCommand);

  digitalWrite(_config.pinConfig.motorDriverDirA, dir);
  digitalWrite(_config.pinConfig.motorDriverDirB, !dir);    
  analogWrite(_config.pinConfig.motorDriverPWM, speed);

}
  */