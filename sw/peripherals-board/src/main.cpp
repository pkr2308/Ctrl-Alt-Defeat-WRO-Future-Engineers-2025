/**
 * @file main.cpp
 * @brief Initialises drivers and managers, handles interaction between managers
 * @author DIY Labs
 */

#include "Arduino.h"

#define VEHICLE_DRIVERSET_HWREV2                 // HWREV2, HWREV1 **NOTE** HWREV1 DRIVERS ARE INCOMPLETE, BUGGY, OR MISSING!!
#define VEHICLE_CONFIG DEBUG_DRIVE_FROM_RADIO    // DEBUG_DRIVE_FROM_RADIO, DEBUG_DRIVE_FROM_SERIAL
#include <driverconfig.hpp>                      // *NOTE* All config #defines must be before this include
#include <SensorManager.hpp>


VEHICLE_DRIVER_IMU bno(VEHICLE_GET_CONFIG);
VEHICLE_DRIVER_LIDAR lidar(VEHICLE_GET_CONFIG);
VEHICLE_DRIVER_SPEED speed(VEHICLE_GET_CONFIG);
VEHICLE_DRIVER_MOTOR motor(VEHICLE_GET_CONFIG);
VEHICLE_DRIVER_STEERING steering(VEHICLE_GET_CONFIG);
VEHICLE_DRIVER_TARGET_CONTROL targetControl(VEHICLE_GET_CONFIG);
SensorManager sensorManager(VEHICLE_GET_CONFIG);

void setup(){

  Serial.begin();

  targetControl.init(&motor, &steering);

  sensorManager.addSensor(&bno);
  sensorManager.addSensor(&lidar);
  sensorManager.addSensor(&speed);
  sensorManager.init();

}

void loop(){

  VehicleData vehicleData = sensorManager.update();

  VehicleCommand cmd;
  cmd.targetYaw = 90;

  targetControl.targetControl(cmd, vehicleData);

}
