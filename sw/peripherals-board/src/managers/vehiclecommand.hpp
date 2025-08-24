/**
 * @brief Struct for storing vehicle commands
 * @author DIY Labs
 * @note Negative values in an *unsigned* int16! Demonic behavior and potentially undefined behavior
 */

#pragma once

#include <cstdint>

enum VehicleInstruction{

  NO_INSTRUCTION,
  RP2040_RPI_START_OBSTACLE_NAVIGATION,
  RPI_RP2040_START_PARKING

};

struct VehicleCommand{

  int16_t targetSpeed; // Note from DIY: Putting negative values into an *unsigned* int16 was not a good idea
  uint16_t targetYaw;
  VehicleInstruction instruction = NO_INSTRUCTION;  // initialised by default to NO_INSTRUCTION

};