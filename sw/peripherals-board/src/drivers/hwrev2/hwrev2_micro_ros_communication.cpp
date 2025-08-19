#include <hwrev2_micro_ros_communication.hpp>

hw_rev_2_ROSCommunication::hw_rev_2_ROSCommunication(VehicleConfig cfg){

  _config = cfg;

}

void hw_rev_2_ROSCommunication::init(ILogger *logger){

  _logger = logger;
  _logger->sendMessage("hw_rev_2_ROSCommunication::init", _logger->INFO, "ROS communication driver initialised");

}

VehicleCommand hw_rev_2_ROSCommunication::update(VehicleData data, VehicleCommand cmd){

  VehicleCommand returnCommand;

  return returnCommand;
  
}