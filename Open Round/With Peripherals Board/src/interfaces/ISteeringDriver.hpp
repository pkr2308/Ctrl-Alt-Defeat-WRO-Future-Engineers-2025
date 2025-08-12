/**
 * @brief Interface for steering driver
 * @author DIY Labs
 */

#pragma once
#include <ILogger.hpp>

class ISteeringDriver{
public:  

  virtual ~ISteeringDriver() = default;

  virtual void init(ILogger *logger) = 0;
  virtual void steer(int steeringAngle) = 0;

};