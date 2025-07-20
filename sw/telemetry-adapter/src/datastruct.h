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

  uint16_t targetYaw;
  uint16_t targetSpeed;

  uint16_t lidarData[3];

  uint8_t bnoSysCal;
  uint8_t bnoGyroCal;
  uint8_t bnoAccelCal;
  uint8_t bnoMagCal;

};