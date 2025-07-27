#pragma once
#include <sensordata.hpp>
#include <sensorstatus.hpp>

class ISensor{
public:  

  virtual ~ISensor() = default;

  virtual void init() = 0;            // todo: return SensorStatus
  virtual SensorData update() = 0;

};