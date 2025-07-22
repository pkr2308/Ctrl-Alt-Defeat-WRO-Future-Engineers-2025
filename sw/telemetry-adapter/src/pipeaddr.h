/*
  pipeaddr.h
  Defines pipe addresses for nRF24L01 radios, shared with vehicle firmware
  Written by DIY Labs

*/
#pragma once

#include <stdint.h>

const uint8_t TELEMETRY_RX_ADDR[] = "TLM00";
const uint8_t VEHICLE_RX_ADDR[] = "CAR00";