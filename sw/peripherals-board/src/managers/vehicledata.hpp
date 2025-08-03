#pragma once

#include "vec3f.hpp"
#include <cstdint>

struct VehicleData{

  Vec3f orientation;
  uint8_t lidar[360];
  uint16_t speed;

};