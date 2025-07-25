/**
 * @file main.cpp
 * @brief Initialises drivers and managers, handles interaction between managers
 * @author DIY Labs
 */

#include "Arduino.h"
#include <configuration.hpp>
#include <PowertrainManager.hpp>

#include <pb_hw_rev_2_includes.hpp>

VehicleConfig config = getVehicleConfig();

PowertrainManager powertrain(config);

void setup(){

}

void loop(){

}