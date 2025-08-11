#pragma once

#include <cstdint>

struct hwrev2_rf24_telem_block1{

  float oriX;
  float oriY;
  float oriZ;

  uint16_t lidarLeft;
  uint16_t lidarFront;
  uint16_t lidarRight;

};

const uint8_t TLM_PIPE_0[] = "TLM00";
const uint8_t CAR_PIPE_0[] = "CAR00";