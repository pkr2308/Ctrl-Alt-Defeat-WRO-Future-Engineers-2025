/**
 * @file main.cpp
 * @brief Initialises drivers and managers, handles interaction between managers
 * @author DIY Labs
 */

#include "Arduino.h"

#define VEHICLE_DRIVERSET_HWREV2                        // HWREV2, HWREV1 **NOTE** HWREV1 DRIVERS ARE INCOMPLETE, BUGGY, OR MISSING!!
#define SINGLE_LIDAR_OPEN_ROUND 
#include <driverconfig.hpp>                             // *NOTE* All config #defines must be before this include
#include <SensorManager.hpp>


VEHICLE_DRIVER_IMU bno(VEHICLE_GET_CONFIG);
VEHICLE_DRIVER_LIDAR lidar(VEHICLE_GET_CONFIG);
VEHICLE_DRIVER_SPEED speed(VEHICLE_GET_CONFIG);
VEHICLE_DRIVER_MOTOR motor(VEHICLE_GET_CONFIG);
VEHICLE_DRIVER_STEERING steering(VEHICLE_GET_CONFIG);
VEHICLE_DRIVER_TARGET_CONTROL targetControl(VEHICLE_GET_CONFIG);
VEHICLE_DRIVE_ALGORITHM driveAlgorithm(VEHICLE_GET_CONFIG);
SensorManager sensorManager(VEHICLE_GET_CONFIG);


void debugPrintVehicleData(VehicleData data);


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

  
}

void loop(){

  VehicleData vehicleData = sensorManager.update();

  VehicleCommand vehicleCommand = driveAlgorithm.drive(vehicleData);

  if(driveAlgorithm.isDirectControl()){
    targetControl.directControl(vehicleCommand, vehicleData);
  }
  else{
    targetControl.targetControl(vehicleCommand, vehicleData);
  }

  debugPrintVehicleData(vehicleData);

  delay(1); // Small delay to allow other tasks to run, and not overwhelm microcontroller
}

void debugPrintVehicleData(VehicleData data){

  Serial.print("Orientation: ");
  Serial.print(data.orientation.x);
  /*
  Serial.print(", ");
  Serial.print(data.orientation.y);
  Serial.print(", ");
  Serial.println(data.orientation.z);
  */

  Serial.print("Speed: ");
  Serial.println(data.speed);

  Serial.print("Distance: ");
  Serial.println(data.encoderPosition/45);

  Serial.print("Servo Position: ");
  Serial.print(' ');

  Serial.print("LiDAR: Left- ");
  Serial.print(data.lidar[270]);
  Serial.print(", Centre- ");
  Serial.print(data.lidar[0]);
  Serial.print(", Right- ");
  Serial.println(data.lidar[90]);

}