/**
 * @brief Interface for sensors
 * @author DIY Labs
 */

#pragma once
#include <sensordata.hpp>
#include <vector>

class ISensor{
public:  

  virtual ~ISensor() = default;

  virtual void init() = 0;            // todo: return status
  virtual std::vector<SensorData> update() = 0;

};