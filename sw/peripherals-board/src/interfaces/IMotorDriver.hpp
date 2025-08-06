/**
 * @brief Interface for motor driver
 * @author DIY Labs
 */

#pragma once

class IMotorDriver{
public:  

  virtual ~IMotorDriver() = default;

  virtual void init() = 0;
  virtual void driveMotor(int speed, bool dir) = 0;
  virtual void disarmMotor() = 0;
  virtual void armMotor() = 0;

};