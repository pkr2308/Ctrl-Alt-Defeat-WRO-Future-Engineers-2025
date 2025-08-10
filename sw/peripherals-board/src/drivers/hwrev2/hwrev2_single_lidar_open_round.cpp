/**
 * @brief Implementation of Single Lidar Open Round drive algorithm
 * @author Pranav
 */

#include "hwrev2_single_lidar_open_round.hpp"
#include <utility/imumaths.h>

hw_rev_2_SingleLidarOpenRound::hw_rev_2_SingleLidarOpenRound(VehicleConfig cfg){
  _config = cfg;
}

void hw_rev_2_SingleLidarOpenRound::init(ILogger* logger) {
  
  _debugLogger = logger;
  _debugLogger->sendMessage("hw_rev_2_SingleLidarOpenRound::init()", _debugLogger->INFO, "Initialising drive algorithm");

  if (front_startDist > 140) stopDist = 0;
  else stopDist = 80;
  
  speed = 225; // Initial speed
  VehicleCommand{.targetSpeed = speed, .targetYaw = 90}; // Set initial speed, steering

}

VehicleCommand hw_rev_2_SingleLidarOpenRound::drive(VehicleData vehicleData){
  
  VehicleCommand command;

  yaw = vehicleData.orientation.x;
  distance = encoderValue / 45;
  
  // Stopping turn logic
  float difference = targetYaw - yaw;
  if (abs(difference) <= 6 && turning == true){   // Return to straight after turning for ~84°
    speed = 225;
    turning = false;
    encoderValue = 0;
    distance = 0;
    turns ++;
    command.targetSpeed = speed;
    pos = 90; // Reset servo position
    command.targetYaw = pos;
    _debugLogger->sendMessage("hw_rev_2_SingleLidarOpenRound::init()", _debugLogger->INFO, "Stopping turn");
  }

  // Starting turn logic
  front_lidarDist = vehicleData.lidar[0];
  if ((front_lidarDist < threshold) && (turning == false) && (turns == 0 or distance > 100)){ // Checking to turn
    speed = 200;
    turning = true;
    command.targetSpeed = speed;
    pos = 90 + turnDir * 42; // Set servo position for turning
    command.targetYaw = pos;
    _debugLogger->sendMessage("hw_rev_2_SingleLidarOpenRound::drive()", _debugLogger->INFO, "Start turn");

  }
  
  // Stop after 3 rounds
  if (turns == 12 && distance >= stopDist){
    completed = true;
    speed = 0; // Stop the vehicle
    command.targetSpeed = speed;
    pos = 90; // Reset servo position
    command.targetYaw = pos;
    _debugLogger->sendMessage("hw_rev_2_SingleLidarOpenRound::init()", _debugLogger->INFO, "Completed 3 rounds");
  }

  // Not turning - Gyro straight follower
  if(turning == false){
    speed = 225;
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

  //if ((turnDir == 0) && (left_lidarDist > left_startDist + 50)) turnDir = -1; // Turning to left
  //else if ((turnDir == 0) && (right_lidarDist > right_startDist + 50)) turnDir = 1; // Turning to right

  return command;

}