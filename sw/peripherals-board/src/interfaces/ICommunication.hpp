/**
  * @brief Interface for two-way communication
  * @author DIY Labs
 */

#pragma once
#include <vehiclecommand.hpp>
#include <vehicledata.hpp>
#include <ILogger.hpp>

class ICommunication{

public:
  virtual void init(ILogger *logger) = 0;
  virtual VehicleCommand update(VehicleData data) = 0;

};