/**
 * @brief Struct for storing data collected from sensors
 * @author DIY Labs
 */

#pragma once

#include "vec3f.hpp"
#include <cstdint>

struct VehicleData{

  Vec3f orientation;
  Vec3f acceleration;
  Vec3f angularVelocity;
  uint16_t lidar[360];
  int16_t speed;
  long encoderPosition;
  
  uint8_t imuCalib;
  uint8_t gyroCalib;
  uint8_t accelCalib;
  uint8_t magCalib;

};