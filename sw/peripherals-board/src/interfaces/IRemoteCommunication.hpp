#pragma once

#include <vehicledata.hpp>

class IRemoteCommunication{
public:  

  virtual ~IRemoteCommunication() = default;

  virtual void init() = 0;
  virtual void steer(int steeringAngle) = 0;

};