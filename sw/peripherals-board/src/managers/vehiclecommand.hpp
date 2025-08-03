#pragma once

#include <cstdint>

struct VehicleCommand{

  uint16_t targetSpeed; // PWM Speed for moor
  uint8_t targetYaw; // Target yaw angle for after steering managed by PID controller

};