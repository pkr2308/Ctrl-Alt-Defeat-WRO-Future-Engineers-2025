#pragma once
#include <sensordata.hpp>

class ISensor{
public:  

  virtual ~ISensor() = default;

  virtual void init() = 0;            // todo: return status
  virtual SensorData update() = 0;

};