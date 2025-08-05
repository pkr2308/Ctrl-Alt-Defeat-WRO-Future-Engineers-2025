#pragma once

#include <cstdint>

struct VehicleCommand{

  int16_t targetSpeed; // Note from DIY: Putting negative values into an *unsigned* int16 is not a good idea
  uint8_t targetYaw;

};