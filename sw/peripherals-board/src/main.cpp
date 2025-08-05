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

  //VehicleCommand vehicleCommand = driveAlgorithm.drive(vehicleData);

  // Temporarily read from radio receiver for testing
  int radioSpeed = map(pulseIn(0, HIGH), 1000, 2000, -1024, 1024);
  int radioSteering = map(pulseIn(1, HIGH), 1000, 2000, 0, 180);

  radioSpeed = constrain(radioSpeed, -1024, 1024);
  radioSteering = constrain(radioSteering, 0, 180);

  VehicleCommand radioCommand;

  radioCommand.targetSpeed = -radioSpeed;
  radioCommand.targetYaw = radioSteering;

  targetControl.directControl(radioCommand, vehicleData);

  Serial.print("Radio Speed: ");
  Serial.print(radioSpeed);
  Serial.print(", Radio Steering: ");
  Serial.println(radioSteering);
/*
  if(driveAlgorithm.isDirectControl()){
    targetControl.directControl(vehicleCommand, vehicleData);
  }
  else{
    targetControl.targetControl(vehicleCommand, vehicleData);
  }
*/
  // debugPrintVehicleData(vehicleData);

}

void debugPrintVehicleData(VehicleData data){

  Serial.print("Orientation: ");
  Serial.print(data.orientation.x);
  Serial.print(", ");
  Serial.print(data.orientation.y);
  Serial.print(", ");
  Serial.println(data.orientation.z);

  Serial.print("Speed: ");
  Serial.println(data.speed);

  Serial.print("Encoder Position: ");
  Serial.println(data.encoderPosition);

  Serial.print("LiDAR: ");
  Serial.print(data.lidar[270]);
  Serial.print(", ");
  Serial.print(data.lidar[0]);
  Serial.print(", ");
  Serial.println(data.lidar[90]);

}