#pragma once

#include <vehicledata.hpp>
#include <vehiclecommand.hpp>
#include <sensorstatus.hpp>

class IRemoteCommunication{
public:  

  virtual ~IRemoteCommunication() = default;

  virtual void init() = 0;
  virtual VehicleCommand update(VehicleData vehicleData);

};