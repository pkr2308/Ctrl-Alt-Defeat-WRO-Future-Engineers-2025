/**
 * @brief Interface for steering driver
 * @author DIY Labs
 */

#pragma once

class ISteeringDriver{
public:  

  virtual ~ISteeringDriver() = default;

  virtual void init() = 0;
  virtual void steer(int steeringAngle) = 0;

};