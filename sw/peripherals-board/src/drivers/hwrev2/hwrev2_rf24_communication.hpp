#pragma once

#include <ICommunication.hpp>
#include <vehiclecommand.hpp>
#include <vehicledata.hpp>
#include <config.hpp>
#include "hwrev2_rf24_shared.hpp"
#include <RF24.h>

class hw_rev_2_RF24Communication : public ICommunication{

public:
  hw_rev_2_RF24Communication(VehicleConfig cfg);
  void init(ILogger *logger) override;
  VehicleCommand update(VehicleData data, VehicleCommand cmd) override;  

private:
  VehicleConfig _config;
  RF24 *_radio;
  ILogger* _logger;

};