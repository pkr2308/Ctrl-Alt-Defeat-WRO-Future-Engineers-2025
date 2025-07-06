/*
  BMP085.h
  Driver for BMP085 barometer
  Written by DIY Labs

*/

#pragma once
#include "Sensor.h"
#include <Adafruit_BMP085.h>
#include <Adafruit_Sensor.h>

class DriverBMP085 : public Sensor{

public:
  DriverBMP085(TwoWire* wire);
  ~DriverBMP085() override = default;

  bool initialize() override;
  std::vector<sensors_event_t> update() override;
  const char* getType() const override { return "BMP085";  }

private:
  Adafruit_BMP085 _bmp;
  TwoWire* _wire;

};

