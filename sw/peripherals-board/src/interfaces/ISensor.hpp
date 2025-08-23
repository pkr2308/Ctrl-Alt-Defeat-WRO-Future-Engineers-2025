/**
 * @brief Interface for sensors
 * @author DIY Labs
 */

#pragma once
#include <sensordata.hpp>
#include <vector>
#include <ILogger.hpp>

class ISensor{
public:  

  virtual ~ISensor() = default;

  virtual void init(ILogger *logger) = 0;            // todo: return status
  virtual std::vector<SensorData> update() = 0;
  virtual String getSensorName() = 0;

};