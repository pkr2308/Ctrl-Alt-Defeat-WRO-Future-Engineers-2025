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
SensorManager sensorManager(VEHICLE_GET_CONFIG);


void debugPrintVehicleData(VehicleData data);

/**
 * @brief Initialises sensor manager, target controller, and drive algorithm
 * @author DIY Labs
 */
void setup(){

  Serial.begin();

  targetControl.init(&motor, &steering);

  driveAlgorithm.init();

  sensorManager.addSensor(&bno);
  sensorManager.addSensor(&lidar);
  sensorManager.addSensor(&speed);
  sensorManager.init();

  

  /*
  while (true){
    int pinValue = digitalRead(startBtn);
    Serial.println("Waiting");
    if(pinValue != 1){
      Serial.print("Started");
      break;
    }
  }
  delay(1500);*/

  pinMode(0, INPUT);
  pinMode(1, INPUT);
  
}

/**
 * @brief Reads data from sensors, passes drive commands from drive algorithm to target controller
 * @author DIY Labs
 */
void loop(){

  VehicleData vehicleData = sensorManager.update();

  VehicleCommand vehicleCommand = driveAlgorithm.drive(vehicleData);

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