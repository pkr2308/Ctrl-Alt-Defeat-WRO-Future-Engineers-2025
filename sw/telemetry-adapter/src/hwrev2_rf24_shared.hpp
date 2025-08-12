#pragma once

#include <cstdint>

struct hwrev2_rf24_telem_block1{

  // 12 bytes
  float oriX;
  float oriY;
  float oriZ;

  // 48 bytes
  uint16_t lidarLeft;
  uint16_t lidarFront;
  uint16_t lidarRight;

  // 32 btyes
  uint16_t commandedSpeed;
  uint16_t commandedSteer;

  // 32 bytes
  uint8_t imuCalib;
  uint8_t gyroCalib;
  uint8_t accelCalib;
  uint8_t magCalib;

  int16_t speed;

};

const uint8_t TLM_PIPE_0[] = "TLM00";
const uint8_t CAR_PIPE_0[] = "CAR00";