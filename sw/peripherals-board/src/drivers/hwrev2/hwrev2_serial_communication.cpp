#include <hwrev2_serial_communication.hpp>

hw_rev_2_SerialCommunication::hw_rev_2_SerialCommunication(VehicleConfig cfg){

  _config = cfg;

}

void hw_rev_2_SerialCommunication::init(ILogger *logger){

  _logger = logger;

  Serial.begin(115200);

}

VehicleCommand hw_rev_2_SerialCommunication::update(VehicleData data, VehicleCommand cmd){

  VehicleCommand returnCommand;

  if(Serial.available()){

    String command = Serial.readStringUntil('\n');

    int commaIndex = command.indexOf(',');

    _targetSpeed = command.substring(0, commaIndex).toInt();
    _targetYaw = command.substring(commaIndex + 1).toInt();

  }

  returnCommand.targetSpeed = _targetSpeed;
  returnCommand.targetYaw = _targetYaw;

  return returnCommand;
  
}