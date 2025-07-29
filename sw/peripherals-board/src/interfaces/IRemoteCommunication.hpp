#pragma once

#include <vehicledata.hpp>
#include <vehiclecommand.hpp>
#include <status.hpp>

class IRemoteCommunication{
public:  

  virtual ~IRemoteCommunication() = default;

  virtual status_t init() = 0;
  virtual VehicleCommand update(VehicleData vehicleData);

};