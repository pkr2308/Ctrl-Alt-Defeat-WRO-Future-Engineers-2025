/*
  TCS34725.h
  Driver for TCS34725 color sensor 
  Written by DIY Labs

*/

#pragma once
#include "Sensor.h"
#include <Adafruit_TCS34725.h>
#include <Adafruit_Sensor.h>

class DriverTCS34725 : public Sensor{

public:
  DriverTCS34725();
  ~DriverTCS34725() override = default;

  bool initialize() override;
  std::vector<sensors_event_t> update() override;
  const char* getType() const override { return "TCS34625";  }

private:
  Adafruit_TCS34725 _tcs;
  TwoWire* _wire;

};

