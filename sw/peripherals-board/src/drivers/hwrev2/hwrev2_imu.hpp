/**
 * @brief Header for hwrev2 IMU driver
 * @author DIY Labs
 */

#pragma once

#include <config.hpp>
#include <ISensor.hpp>
#include <sensordata.hpp>
#include <Adafruit_BNO055.h>

class hw_rev_2_imu : public ISensor{
public:
  hw_rev_2_imu(VehicleConfig cfg);
  void init() override;
  SensorData update() override;

private:
  VehicleConfig _config;
  Adafruit_BNO055 *_bno;

};