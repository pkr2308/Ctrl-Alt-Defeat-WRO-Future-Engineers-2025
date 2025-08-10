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
VEHICLE_DRIVER_DEBUG_LOG debugLogger(VEHICLE_GET_CONFIG);
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

  Serial1.setRX(VEHICLE_GET_CONFIG.pinConfig.uart0RX);
  Serial1.setTX(VEHICLE_GET_CONFIG.pinConfig.uart0TX);  

  debugLogger.init();  

  Serial.begin();

  targetControl.init(&motor, &steering, &debugLogger);
  driveAlgorithm.init(&debugLogger);

  sensorManager.addSensor(&bno);
  sensorManager.addSensor(&lidar);
  sensorManager.addSensor(&speed);
  sensorManager.init(&debugLogger);

  remoteCommunication.init(&debugLogger);
  
}

/**
 * @brief Reads data from sensors, passes drive commands from drive algorithm/RPi/radio to target controller
 */
void loop(){

  VehicleData vehicleData = sensorManager.update();

  VehicleCommand driveAlgorithmCommand = driveAlgorithm.drive(vehicleData);
  VehicleCommand radioCommand = remoteCommunication.update(vehicleData);

  debugPrintVehicleData(vehicleData);

  delay(1);
}

/**
 * @brief Function for printing all collected vehicle data without names for use with SerialPlot
 * @note Do not modify order or add name print to variables
 */
void debugPrintVehicleData(VehicleData data){

  Serial.print("Yaw: ");
  Serial.print(data.orientation.x);
  Serial.print(" Pitch: ");
  Serial.print(-data.orientation.y);
  Serial.print(" Roll: ");
  Serial.print(data.orientation.z);
  Serial.print(" Accel X: ");

  Serial.print(data.acceleration.x);
  Serial.print(" Accel Y: ");
  Serial.print(data.acceleration.y);
  Serial.print(" Accel Z: ");
  Serial.print(data.acceleration.z);
  Serial.print(" Angular X: ");

  Serial.print(data.angularVelocity.x);
  Serial.print(" Angular Y: ");
  Serial.print(data.angularVelocity.y);
  Serial.print(" Angular Z: ");
  Serial.println(data.angularVelocity.z);
  Serial.print("Encoder: ");

  Serial.print(data.encoderPosition);
  Serial.print(" Speed: ");

  Serial.println(data.speed);
  Serial.print("Lidar Left: ");

  Serial.print(data.lidar[270]);
  Serial.print(" Front: ");
  Serial.print(data.lidar[0]);
  Serial.print(" Right: ");
  Serial.print(data.lidar[90]);
  Serial.print(" ");

  Serial.println();

}