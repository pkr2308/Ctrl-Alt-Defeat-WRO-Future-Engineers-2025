/**
 * @brief Implementation of Single Lidar Open Round drive algorithm
 * @author Pranav
 */

#include "hwrev2_single_lidar_open_round.hpp"
#include <utility/imumaths.h>

hw_rev_2_SingleLidarOpenRound::hw_rev_2_SingleLidarOpenRound(VehicleConfig cfg){
  _config = cfg;
}

void hw_rev_2_SingleLidarOpenRound::init() {
  
  
}

VehicleCommand hw_rev_2_SingleLidarOpenRound::drive(VehicleData vehicleData){
  
  VehicleCommand command;

  yaw = vehicleData.orientation.x;
  distance = encoderValue / 45;
  
  // Stopping turn logic
  float difference = targetYaw - yaw;
  if (abs(difference) <= 6 && turning == true){   // Return to straight after turning for ~84°
    speed = 275;
    turning = false;
    encoderValue = 0;
    distance = 0;
    turns ++;
    command.targetSpeed = speed;
    pos = 90; // Reset servo position
    command.targetYaw = pos;
  }

  // Starting turn logic
  lidarDist = vehicleData.lidar[0];
  if ((lidarDist < threshold) && (turning == false) && (turns == 0 or distance > 100)){ // Checking to turn
    speed = 225;
    turning = true;
    command.targetSpeed = speed;
    pos = 90 + turnDir * 42; // Set servo position for turning
    command.targetYaw = pos;
  }
  
  // Stop after 3 rounds
  if (turns == 12 && distance >= stopDist){
    completed = true;
    speed = 0; // Stop the vehicle
    command.targetSpeed = speed;
    pos = 90; // Reset servo position
    command.targetYaw = pos;
  }

  // Not turning - Gyro straight follower
  if(turning == false){
    speed = 275;
    correction = 0;
    error = round(targetYaw - yaw);
    if (error > 180) error = error - 360;
    else if (error < -180) error = error + 360;
    totalError += error;
    if (error > 0) correction = error * 2.3 - totalError * 0.001; // correction to the right
    else if (error < 0) correction = error * 2.2 - totalError * 0.001; // correction to the left

    if (correction > 45) correction = 45;
    else if (correction < -45) correction = -45;
    command.targetSpeed = speed;
    command.targetYaw = 90 + correction; // Set servo position based on correction
    
  } 

  return command;

}