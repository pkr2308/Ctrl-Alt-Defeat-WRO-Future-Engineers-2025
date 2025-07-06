/*
  MPU6050.h
  Driver for MPU6050 IMU 
  Written by DIY Labs

*/

#pragma once
#include "Sensor.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

class DriverMPU6050 : public Sensor{

public:
  DriverMPU6050(TwoWire* wire);
  ~DriverMPU6050() override = default;

  bool initialize() override;
  std::vector<sensors_event_t> update() override;
  const char* getType() const override { return "MPU6050";  }

private:
  Adafruit_MPU6050 _mpu;
  TwoWire* _wire;

};

