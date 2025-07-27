/**
 * @file main.cpp
 * @brief Initialises drivers and managers, handles interaction between managers
 * @author DIY Labs
 */

#include "Arduino.h"
#include <hwrev1_motor_driver.hpp>
#include <hwrev1_get_config.hpp>

hw_rev_1_MotorDriver motorDriver(hwrev1_getConfig());

void setup(){

  motorDriver.init();
  motorDriver.armMotor();
  motorDriver.driveMotor(512, true);

}

void loop(){

}