#include <hwrev2_rf24_communication.hpp>

hw_rev_2_RF24Communication::hw_rev_2_RF24Communication(VehicleConfig cfg){

  _config = cfg;

  _radio = new RF24(_config.pinConfig.nrfCS, _config.pinConfig.nrfCE);

}

void hw_rev_2_RF24Communication::init(){

  Serial.println(_radio->begin());

}

VehicleCommand hw_rev_2_RF24Communication::update(VehicleData data){

  VehicleCommand cmd;
  return cmd;
  
}