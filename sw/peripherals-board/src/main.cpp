// TODO: Figure out a clean way of switching between target control sources, ICommunication and IDriveAlgorithm, combine?k
// Maybe implement radio and serial as drive algorithms, how to pass comm drivers to them?

/**
 * @file main.cpp
 * @brief Initialises drivers and managers, handles interaction between managers
 * @author DIY Labs
 */

#include "Arduino.h"

#define VEHICLE_DRIVERSET_HWREV2                        // HWREV2 **NOTE** HWREV1 DRIVERS ARE INCOMPLETE, BUGGY, OR MISSING!!
#define OPEN_ROUND                                      // Defines what drive algorithm to use. Add more in driverconfig.hpp
#define VEHICLE_SW_STATUS "DEV"                         // String containing status of software. Printed over debug port
#define VEHICLE_SW_NAME "ROS Communication Trial"       // String containing status of software. Printed over debug port

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
VEHICLE_DRIVER_SERIAL_COMMUNICATION serialCommunication(VEHICLE_GET_CONFIG);
VEHICLE_DRIVER_ROS_COMMUNICATION rosCommunication(VEHICLE_GET_CONFIG);
VEHICLE_DRIVER_DEBUG_LOG debugLogger(VEHICLE_GET_CONFIG);
SensorManager sensorManager(VEHICLE_GET_CONFIG);
VehicleCommand activeDriveCommand;


void debugPrintVehicleData(VehicleData data, VehicleCommand cmd);
void debugLogHeader();
void debugLogDataCommand(VehicleData data, VehicleCommand cmd);


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
  debugLogHeader();


  Serial.begin();

  targetControl.init(&motor, &steering, &debugLogger);
  driveAlgorithm.init(&debugLogger);

  delay(2000);

  sensorManager.addSensor(&bno);
  sensorManager.addSensor(&lidar);
  sensorManager.addSensor(&speed);
  sensorManager.init(&debugLogger);

  remoteCommunication.init(&debugLogger);
  serialCommunication.init(&debugLogger);
  rosCommunication.init(&debugLogger);
  
}

/**
 * @brief Reads data from sensors, passes drive commands from drive algorithm/RPi/radio to target controller
 */
void loop(){

  VehicleData vehicleData = sensorManager.update();

  VehicleCommand remoteCommunicationCommand = remoteCommunication.update(vehicleData, activeDriveCommand);
  VehicleCommand serialCommunicationCommand = serialCommunication.update(vehicleData, activeDriveCommand);
  VehicleCommand rosCommunicationCommand    = rosCommunication.update(vehicleData, activeDriveCommand);

  targetControl.directControl(activeDriveCommand, vehicleData);

}

/**
 * @brief Function for printing all collected vehicle data without names for use with SerialPlot
 * @note Do not modify order or add name print to variables 
 */
void debugPrintVehicleData(VehicleData data, VehicleCommand cmd){

  Serial.print("Yaw: ");
  Serial.print(data.orientation.x);
  /*Serial.print(" Pitch: ");
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
  /*Serial.print(" Angular Y: ");
  Serial.print(data.angularVelocity.y);
  Serial.print(" Angular Z: ");
  Serial.println(data.angularVelocity.z);*/
  
  Serial.print(" Encoder: ");
  Serial.print(data.encoderPosition);
  Serial.print(" Distance: ");
  Serial.print(data.encoderPosition / 43);
  Serial.print(" Speed: ");
  Serial.println(data.speed);

  Serial.print(" Lidar Left: ");
  Serial.print(data.lidar[270]);
  Serial.print(" Front: ");
  Serial.print(data.lidar[0]);
  Serial.print(" Right: ");
  Serial.print(data.lidar[90]);
  Serial.print(" ");

  Serial.print(" Target Servo ");
  Serial.print(cmd.targetYaw);
  Serial.println();

}

void debugLogDataCommand(VehicleData data, VehicleCommand cmd){

  debugLogger.sendString(String(data.orientation.x));
  debugLogger.sendString(", ");
  debugLogger.sendString(String(data.orientation.y));
  debugLogger.sendString(", ");
  debugLogger.sendString(String(data.orientation.z));
  debugLogger.sendString(", ");

  debugLogger.sendString(String(data.acceleration.x));
  debugLogger.sendString(", ");
  debugLogger.sendString(String(data.acceleration.y));
  debugLogger.sendString(", ");
  debugLogger.sendString(String(data.acceleration.z));
  debugLogger.sendString(", ");

  debugLogger.sendString(String(data.angularVelocity.x));
  debugLogger.sendString(", ");
  debugLogger.sendString(String(data.angularVelocity.y));
  debugLogger.sendString(", ");
  debugLogger.sendString(String(data.angularVelocity.z));
  debugLogger.sendString(", ");

  debugLogger.sendString(String(data.encoderPosition));
  debugLogger.sendString(", ");

  debugLogger.sendString(String(data.speed));
  debugLogger.sendString(", ");

  debugLogger.sendString(String(data.lidar[270]));
  debugLogger.sendString(", ");
  debugLogger.sendString(String(data.lidar[0]));
  debugLogger.sendString(", ");
  debugLogger.sendString(String(data.lidar[90]));
  debugLogger.sendString(", ");

  debugLogger.sendString(String(cmd.targetSpeed));
  debugLogger.sendString(", ");
  debugLogger.sendString(String(cmd.targetYaw));
  debugLogger.sendString(", ");

  debugLogger.sendString("\n");

}

void debugLogHeader(){

  debugLogger.sendMessage("debugLogHeader()", debugLogger.INFO, "CTRL+ALT+DEFEAT Peripherals Board Debug Port");
  debugLogger.sendMessage("debugLogHeader()", debugLogger.INFO, "Software status: " + String(VEHICLE_SW_STATUS));
  debugLogger.sendMessage("debugLogHeader()", debugLogger.INFO, "Software name: " + String(VEHICLE_SW_NAME));  
  debugLogger.sendMessage("debugLogHeader()", debugLogger.INFO, "Compiled on: " + String(__DATE__) + " at: " + String(__TIME__));
  debugLogger.sendMessage("debugLogHeader()", debugLogger.INFO, "Pico SDK version: " + String(PICO_SDK_VERSION_STRING));

}