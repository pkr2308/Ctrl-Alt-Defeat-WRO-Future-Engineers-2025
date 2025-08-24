#include <hwrev2_unpark.hpp>

hw_rev_2_UnparkAlgorithm::hw_rev_2_UnparkAlgorithm(VehicleConfig cfg){
 
  _config = cfg;

}

void hw_rev_2_UnparkAlgorithm::init(ILogger *logger){

  _logger = logger;

}

VehicleCommand hw_rev_2_UnparkAlgorithm::drive(VehicleData data){

  VehicleCommand cmd;

  cmd.targetSpeed = 200;
  cmd.targetYaw = 20;

  return cmd;

}