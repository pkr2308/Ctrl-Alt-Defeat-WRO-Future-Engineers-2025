/**
 * @brief Header for hwrev2 vehicle speed sensor driver
 * @author DIY Labs
 */

#pragma once

#include <config.hpp>
#include <ISensor.hpp>
#include <sensordata.hpp>
#include <RotaryEncoder.h>
#include <vector>

class hw_rev_2_VehicleSpeed : public ISensor{
public:
  hw_rev_2_VehicleSpeed(VehicleConfig cfg);
  void init() override;
  std::vector<SensorData> update() override;
  void _encoderISR();

private:
  VehicleConfig _config;
  RotaryEncoder *_encoder;
  long _encoderPosition; 
  long _prevEncoderPosition; 
  unsigned long lastUpdateTime = 0;
  long prevEncoderPosition = 0;
  uint16_t _speed; 
  uint16_t _ticksPerCM;

};