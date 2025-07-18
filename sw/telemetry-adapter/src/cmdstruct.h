/*
  cmdstruct.h
  Command struct, shared with vehicle firmware

  Written by DIY Labs

*/
#pragma once

#include <stdint.h>

struct RFCommand{

  uint16_t targetSpeed;
  uint16_t targetHeading;  

};