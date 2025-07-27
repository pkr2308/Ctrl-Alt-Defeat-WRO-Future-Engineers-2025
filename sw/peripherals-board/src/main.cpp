/**
 * @file main.cpp
 * @brief Initialises drivers and managers, handles interaction between managers
 * @author DIY Labs
 */

#include "Arduino.h"
#include <hwrev1_get_config.hpp>
#include <hwrev1_steering_driver.hpp>

hw_rev_1_SteeringDriver steeringDriver(hwrev1_getConfig());

void setup(){

  steeringDriver.init();
  steeringDriver.steer(-90);
  delay(500);
  steeringDriver.steer(90);
  delay(500);
  steeringDriver.steer(0);
  delay(500);

}

void loop(){

}