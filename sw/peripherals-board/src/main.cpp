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
#define VEHICLE_SW_NAME "Parking Module Tests"          // String containing status of software. Printed over debug port

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
//VEHICLE_DRIVER_ROS_COMMUNICATION rosCommunication(VEHICLE_GET_CONFIG);
VEHICLE_DRIVER_DEBUG_LOG debugLogger(VEHICLE_GET_CONFIG);
VEHICLE_DRIVER_RGB_LED rgbLED(VEHICLE_GET_CONFIG);
SensorManager sensorManager(VEHICLE_GET_CONFIG);
VehicleCommand activeDriveCommand;


void debugPrintVehicleData(VehicleData data, VehicleCommand cmd);
void debugLogHeader();
void debugLogDataCommand(VehicleData data, VehicleCommand cmd);
void debugProbeI2CAddr(byte addr);

hw_rev_2_Park park(VEHICLE_GET_CONFIG);

/**
 * @brief Initialises sensor manager, target controller, and drive algorithm
 */
void setup(){

  debugLogger.init();  
  debugLogHeader();

  SPI1.setSCK(VEHICLE_GET_CONFIG.pinConfig.spi1SCK);
  SPI1.setRX(VEHICLE_GET_CONFIG.pinConfig.spi1MISO);
  SPI1.setTX(VEHICLE_GET_CONFIG.pinConfig.spi1MOSI);
  SPI1.begin();

  Serial1.setRX(VEHICLE_GET_CONFIG.pinConfig.uart0RX);
  Serial1.setTX(VEHICLE_GET_CONFIG.pinConfig.uart0TX);  

  Wire.setSCL(VEHICLE_GET_CONFIG.pinConfig.i2c0SCL);
  Wire.setSDA(VEHICLE_GET_CONFIG.pinConfig.i2c0SDA);

  debugLogger.sendMessage("setup()", debugLogger.INFO, "Finished setting communication pins for SPI1, UART1, I2C0");

//  driveAlgorithm.init(&debugLogger);
//  park.init(&debugLogger, false);

  sensorManager.init(&debugLogger);
  sensorManager.addSensor(&bno);
  sensorManager.addSensor(&lidar);
  sensorManager.addSensor(&speed);
  debugLogger.sendMessage("setup()", debugLogger.INFO, "Finished adding drivers to sensor manager");

  targetControl.init(&motor, &steering, &debugLogger);

  remoteCommunication.init(&debugLogger);
//  serialCommunication.init(&debugLogger);

  rgbLED.init(&debugLogger);
  rgbLED.limitBrightness(50);
  rgbLED.setStaticColor(rgbLED.GREEN);
  
}

/**
 * @brief Reads data from sensors, passes drive commands from drive algorithm/RPi/radio to target controller
 */
void loop(){

  VehicleData vehicleData = sensorManager.update();

  remoteCommunication.update(vehicleData, activeDriveCommand);    // Send data over nRF24L01+, ignore any commands from telemetry module
  //VehicleCommand serialCommunicationCommand = serialCommunication.update(vehicleData, activeDriveCommand);
//  VehicleCommand parkCommand = park.drive(vehicleData);
    
//  activeDriveCommand = parkCommand;
//  targetControl.directControl(activeDriveCommand, vehicleData);

  debugLogDataCommand(vehicleData, activeDriveCommand);

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

  debugLogger.sendMessage("debugLogDataCommand()", debugLogger.INFO, "Data frame start");

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

  debugLogger.sendString(String(data.lidar[0]));
  debugLogger.sendString(", ");
  debugLogger.sendString(String(data.lidar[90]));
  debugLogger.sendString(", ");
  debugLogger.sendString(String(data.lidar[180]));
  debugLogger.sendString(", ");
  debugLogger.sendString(String(data.lidar[270]));
  debugLogger.sendString(", ");

  debugLogger.sendString(String(cmd.targetSpeed));
  debugLogger.sendString(", ");
  debugLogger.sendString(String(cmd.targetYaw));
  debugLogger.sendString(", ");

  debugLogger.sendString("\n");

  debugLogger.sendMessage("debugLogDataCommand()", debugLogger.INFO, "Data frame end");

}

void debugLogHeader(){

  debugLogger.sendMessage("debugLogHeader()", debugLogger.INFO, "CTRL+ALT+DEFEAT Peripherals Board Debug Port");
  debugLogger.sendMessage("debugLogHeader()", debugLogger.INFO, "Software status: " + String(VEHICLE_SW_STATUS));
  debugLogger.sendMessage("debugLogHeader()", debugLogger.INFO, "Software name: " + String(VEHICLE_SW_NAME));  
  debugLogger.sendMessage("debugLogHeader()", debugLogger.INFO, "Compiled on: " + String(__DATE__) + " at: " + String(__TIME__));
  debugLogger.sendMessage("debugLogHeader()", debugLogger.INFO, "Pico SDK version: " + String(PICO_SDK_VERSION_STRING));

}

/**
 * @note Sometimes doesn't return a success, even when device works. Don't have time to debug it now. Check data stream over debug to ensure sensors are alive.
 */
void debugProbeI2CAddr(byte addr){
    
  Wire.beginTransmission(addr);
  byte err = Wire.endTransmission();

  String errStr;

  switch(err){
    case 0:
      errStr = "Success";
      break;

    case 1:
      errStr = "Data too long";
      break;

    case 2:
      errStr = "NACK on transmit of address";
      break;

    case 3:
      errStr = "NACK on transmit of data";
      break;

    case 4:
      errStr = "Other error";
      break;

    case 5:
      errStr = "Timeout";
      break;

    default:
      break;
    
  };

  debugLogger.sendMessage("debugLogHeader()", debugLogger.INFO, "Error for address " + String(addr, HEX) + " is " + errStr);

  delay(5);

}