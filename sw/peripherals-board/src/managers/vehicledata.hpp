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
  uint8_t lidar[360];
  uint16_t speed;
  long encoderPosition;

};