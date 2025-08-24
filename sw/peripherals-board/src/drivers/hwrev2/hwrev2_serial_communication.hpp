#pragma once

#include <ICommunication.hpp>
#include <vehiclecommand.hpp>
#include <vehicledata.hpp>
#include <config.hpp>
#include <scheduler.hpp>

class hw_rev_2_SerialCommunication : public ICommunication{

public:
  hw_rev_2_SerialCommunication(VehicleConfig cfg);
  void init(ILogger *logger) override;
  VehicleCommand update(VehicleData data, VehicleCommand cmd) override;  

private:
  VehicleConfig _config;
  ILogger* _logger;
  int16_t _targetSpeed; 
  uint16_t _targetYaw;  
  VehicleInstruction _instruction;
  void _serialCallback();
  void _sendFormattedData(VehicleData data);
  static void _serialCallbackWrapper();
  SchedulerTask *_serialTask;

};