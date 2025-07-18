/*
  datastruct.h
  Data struct, shared with vehicle firmware

  Written by DIY Labs

*/
#pragma once

#include <stdint.h>
#include "vector.h"

struct RFData{

  Vec3f orientation;
  uint8_t steeringAngle;
  uint16_t speed;

};