/*
  TFLuna.h
  Driver for TFLuna LiDAR
  Written by DIY Labs

*/

#pragma once

#include "Sensor.h"
#include <TFLI2C.h>
#include <Adafruit_Sensor.h>

class DriverTFLuna : public Sensor{

public:
  DriverTFLuna();
  ~DriverTFLuna() override = default;

  bool initialize() override;
  std::vector<sensors_event_t> update() override;
  const char* getType() const override { return "TFLuna";  }

private:
  TFLI2C _tfluna;

};

