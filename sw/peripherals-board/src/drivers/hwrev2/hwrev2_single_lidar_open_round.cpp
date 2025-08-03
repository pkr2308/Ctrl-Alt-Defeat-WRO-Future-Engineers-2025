#include "hwrev2_single_lidar_open_round.hpp"

hw_rev_2_SingleLidarOpenRound::hw_rev_2_SingleLidarOpenRound(VehicleConfig cfg){
  _config = cfg;
}

void hw_rev_2_SingleLidarOpenRound::init() {

}

VehicleCommand hw_rev_2_SingleLidarOpenRound::drive(VehicleData vehicleData){
    // vehicleData contains orientation and LiDAR data.
    // LiDAR data is an array of 360 integers. Three TFLunas are mapped to 270, 0, and 90.
    // VehicleCommand contains targetSpeed and targetYaw. Functions and variables can be added to hwrev2_single_lidar_open_round.hpp

  VehicleCommand command;
  
  return command;

}