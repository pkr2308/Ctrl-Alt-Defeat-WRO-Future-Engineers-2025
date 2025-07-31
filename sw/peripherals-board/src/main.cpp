/**
 * @file main.cpp
 * @brief Initialises drivers and managers, handles interaction between managers
 * @author DIY Labs
 */

#include "Arduino.h"

#define VEHICLE_DRIVERSET_HWREV1
#include <driverconfig.hpp>

VEHICLE_DRIVER_MOTOR motorDriver(VEHICLE_GET_CONFIG);
VEHICLE_DRIVER_STEERING steeringDriver(VEHICLE_GET_CONFIG);
VEHICLE_DRIVER_TARGET_CONTROL targetControl(VEHICLE_GET_CONFIG);

void setup(){

  targetControl.init(&motorDriver, &steeringDriver);

}

void loop(){

}
