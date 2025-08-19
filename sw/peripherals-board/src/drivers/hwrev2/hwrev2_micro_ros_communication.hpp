#pragma once

#include <ICommunication.hpp>
#include <vehiclecommand.hpp>
#include <vehicledata.hpp>
#include <config.hpp>

class hw_rev_2_ROSCommunication : public ICommunication{

public:
  hw_rev_2_ROSCommunication(VehicleConfig cfg);
  void init(ILogger *logger) override;
  VehicleCommand update(VehicleData data, VehicleCommand cmd) override;  

private:
  VehicleConfig _config;
  ILogger* _logger;

};