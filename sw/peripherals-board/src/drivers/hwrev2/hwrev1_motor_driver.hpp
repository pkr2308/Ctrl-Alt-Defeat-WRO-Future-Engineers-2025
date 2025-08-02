#pragma once

#include <interfaces/IMotorDriver.hpp>
#include <managers/config.hpp>
#include <Arduino.h>

class hw_rev_1_MotorDriver : public IMotorDriver{
public:
  hw_rev_1_MotorDriver(VehicleConfig cfg);

  void init() override;
  void driveMotor(int speed, bool dir) override;
  void disarmMotor() override;
  void armMotor() override;

private:
  VehicleConfig _config;
  const int absMaxMotorCommand = 1024;

};