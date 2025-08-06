/**
 * @file main.cpp
 * @brief Initialises drivers and managers, handles interaction between managers
 * @author DIY Labs
 */

#include "Arduino.h"

#define VEHICLE_DRIVERSET_HWREV2                        // HWREV2 **NOTE** HWREV1 DRIVERS ARE INCOMPLETE, BUGGY, OR MISSING!!
#define SINGLE_LIDAR_OPEN_ROUND                         // Defines what drive algorithm to use. Add more in driverconfig.hpp     
#include <driverconfig.hpp>                             // **NOTE** All config #defines must be before this include
#include <SensorManager.hpp>


VEHICLE_DRIVER_IMU bno(VEHICLE_GET_CONFIG);
VEHICLE_DRIVER_LIDAR lidar(VEHICLE_GET_CONFIG);
VEHICLE_DRIVER_SPEED speed(VEHICLE_GET_CONFIG);
VEHICLE_DRIVER_MOTOR motor(VEHICLE_GET_CONFIG);
VEHICLE_DRIVER_STEERING steering(VEHICLE_GET_CONFIG);
VEHICLE_DRIVER_TARGET_CONTROL targetControl(VEHICLE_GET_CONFIG);
VEHICLE_DRIVER_DRIVE_ALGORITHM driveAlgorithm(VEHICLE_GET_CONFIG);
VEHICLE_DRIVER_REMOTE_COMMUNICATION remoteCommunication(VEHICLE_GET_CONFIG);
SensorManager sensorManager(VEHICLE_GET_CONFIG);


void debugPrintVehicleData(VehicleData data);

/**
 * @brief Initialises sensor manager, target controller, and drive algorithm
 */
void setup(){

  SPI1.setSCK(VEHICLE_GET_CONFIG.pinConfig.spi1SCK);
  SPI1.setRX(VEHICLE_GET_CONFIG.pinConfig.spi1MISO);
  SPI1.setTX(VEHICLE_GET_CONFIG.pinConfig.spi1MOSI);
  SPI1.begin();

  Serial.begin();

  targetControl.init(&motor, &steering);
  driveAlgorithm.init();

  sensorManager.addSensor(&bno);
  sensorManager.addSensor(&lidar);
  sensorManager.addSensor(&speed);
  sensorManager.init();

  remoteCommunication.init();
  
}

/**
 * @brief Reads data from sensors, passes drive commands from drive algorithm to target controller
 */
void loop(){

  VehicleData vehicleData = sensorManager.update();

  //VehicleCommand vehicleCommand = driveAlgorithm.drive(vehicleData);

  remoteCommunication.update(vehicleData);

  debugPrintVehicleData(vehicleData);

}

void debugPrintVehicleData(VehicleData data){

  Serial.print(data.orientation.x);
  Serial.print(", ");
  Serial.print(data.orientation.y);
  Serial.print(", ");
  Serial.print(data.orientation.z);
  Serial.print(", ");

  Serial.print(data.acceleration.x);
  Serial.print(", ");
  Serial.print(data.acceleration.y);
  Serial.print(", ");
  Serial.print(data.acceleration.z);
  Serial.print(", ");

  Serial.print(data.angularVelocity.x);
  Serial.print(", ");
  Serial.print(data.angularVelocity.y);
  Serial.print(", ");
  Serial.print(data.angularVelocity.z);
  Serial.print(", ");

  Serial.print(data.encoderPosition);
  Serial.print(", ");

  Serial.print(data.lidar[270]);
  Serial.print(", ");
  Serial.print(data.lidar[0]);
  Serial.print(", ");
  Serial.print(data.lidar[90]);
  Serial.print(", ");

  Serial.println();

}