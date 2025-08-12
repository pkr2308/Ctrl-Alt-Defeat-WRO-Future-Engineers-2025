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
  
  speed = 225;                      // Initial speed

  VehicleCommand{.targetSpeed = speed, .targetYaw = 90}; // Set initial speed, steering

}

VehicleCommand hw_rev_2_SingleLidarOpenRound::drive(VehicleData vehicleData){
  
  VehicleCommand command;

  // Get yaw and distance from vehicle data
  yaw = vehicleData.orientation.x;
  distance = vehicleData.encoderPosition / 43;
  
  // Stopping turn logic
  float difference = turnDir * (targetYaw - yaw);
  if (difference > 180) difference = 360 - difference;
  else if (difference < -180) difference = 360 + difference;
  if (turning == true){
    if (abs(difference) <= 1){   // Return to straight after turning for ~89°
      speed = 225;
      turning = false;
      encoderValue = 0;
      distance = 0;
      turns += 1;
      if (turnDir == 1){
          targetYaw = targetYaw - turnDir*turns*0.6; // Adjust targetYaw for next turn
          threshold -= 0.8; // Decrease threshold for next turn, there are drifting tendencies; Ref Logs
      }
      else if (turnDir == -1){
          targetYaw = targetYaw + turnDir*turns*0.4; // Adjust targetYaw for next turn
      }
      pos = 90; // Reset servo position
      command.targetYaw = pos;
      _debugLogger->sendMessage("hw_rev_2_SingleLidarOpenRound::drive()", _debugLogger->INFO, "Stopping turn; target " + String(targetYaw) + "; Yaw" + String(yaw));   
    }
    else if (abs(difference) > 10 && abs(difference) < 75){ // Continue turning
      if(turnDir == 1)pos = 90 + (47 - (90-abs(difference))/3); // Set servo position for turning
      else if (turnDir == -1) pos = 90 - (54 - (90-abs(difference))/3.4); // Set servo position for turning
      _debugLogger->sendMessage("hw_rev_2_SingleLidarOpenRound::drive()", _debugLogger->INFO, "Steering " + String(pos) + " Yaw " + String(yaw));
    }
  }

  // Get data from LiDARs
  front_lidarDist = vehicleData.lidar[0];
  left_lidarDist = vehicleData.lidar[270];
  right_lidarDist = vehicleData.lidar[90];

  // Logic to start turning
  if ((front_lidarDist < threshold) and (turning == false) and (turns == 0 or distance > 115) and (abs(difference) < 5) and (left_lidarDist + right_lidarDist > 120)){ // Checking to turn
    speed = 190;                  // Reduce speed for turn
    if (turns == 11) speed = 180; // Slow down further for final turn
    turning = true;

    if (turnDir == 1) pos = 90 + 47; // Set servo position for turning
    else if (turnDir == -1) pos = 90 - 55;

    targetYaw = yaw + turnDir * 90;
    if (targetYaw > 360) targetYaw = targetYaw - 360;
    else if (targetYaw < 0) targetYaw = 360 + targetYaw;

    if ((turns % 4 == 0) and (turnDir == -1)) targetYaw = 270; // First turn in anticlockwise

    if (targetYaw > 75 && targetYaw < 105) targetYaw = 90;
    else if (targetYaw > 165 && targetYaw < 195) targetYaw = 180;
    else if (targetYaw > 255 && targetYaw < 290) targetYaw = 270;
    else if (targetYaw > 345 or targetYaw < 15) targetYaw = 0;
    _debugLogger->sendMessage("hw_rev_2_SingleLidarOpenRound::drive()", _debugLogger->INFO, "Starting turn");

  }

  // Not turning - IMU-based straight follower
  if(turning == false){
    speed = 225;
    correction = 0;
    error = round(targetYaw - yaw);

    if (error > 180) error = error - 360;
    else if (error < -180) error = error + 360;
    totalError += error;                            // Used for integral control

    if (error > 0) correction = error * 2.3 - totalError * 0.001; // correction to the right
    else if (error < 0) correction = error * 2.2 - totalError * 0.001; // correction to the left

    if (correction > 45) correction = 45;
    else if (correction < -45) correction = -45;
    pos = 90 + correction; // Set servo position based on correction

  }

  // Determine turn direction (clockwise/anticlockwise) based on side LiDARs
  // For smoother turns different values of threshold are set
  // The steering is off-centre, so the robot turns more easily to the right
  if (turnDir == 0){
    if (left_lidarDist - right_lidarDist > 100){ 
      turnDir = -1; // Turning to left
      threshold = 78;
    }
    else if (left_lidarDist - right_lidarDist < -100){ 
      turnDir = 1; // Turning to right
      threshold = 74;
    }
  }

  // Stopping after final turn i.e. 3 rounds
  if (turns == 12){
    completed = true;
    speed = 0; // Stop the vehicle
    pos = 90;  // Reset servo position
    _debugLogger->sendMessage("hw_rev_2_SingleLidarOpenRound::drive()", _debugLogger->INFO, "Completed 3 rounds");
  }

  command.targetSpeed = speed;
  
  // This sets servo position not yaw since this system is currently on direct control
  // Ref .hpp file with line 19 - bool isDirectControl() override { return true; }
  command.targetYaw = int(pos);

  return command;

}