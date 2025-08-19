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

    _targetSpeed = constrain(command.substring(0, commaIndex).toInt(), -1024, 1024);
    _targetYaw = constrain(command.substring(commaIndex + 1).toInt(), 0, 180);

    _logger->sendMessage("hw_rev_2_SerialCommunication::update", _logger->INFO, "Received command over USB. Raw: " + command + "_targetSpeed: " + String(_targetSpeed) + " _targetYaw: " + String(_targetYaw));

    _sendFormattedData(data);

  }

  returnCommand.targetSpeed = _targetSpeed;
  returnCommand.targetYaw = _targetYaw;

  return returnCommand;
  
}

void hw_rev_2_SerialCommunication::_sendFormattedData(VehicleData data){

  const char seperator = ',';

  String message;

  message += String(data.orientation.x);
  message += seperator;
  message += String(data.orientation.y);
  message += seperator;
  message += String(data.orientation.z);
  message += seperator;

  message += String(data.acceleration.x);
  message += seperator;
  message += String(data.acceleration.y);
  message += seperator;
  message += String(data.acceleration.z);
  message += seperator;

  message += String(data.angularVelocity.x);
  message += seperator;
  message += String(data.angularVelocity.y);
  message += seperator;
  message += String(data.angularVelocity.z);
  message += seperator;

  message += String(data.lidar[0]);
  message += seperator;
  message += String(data.lidar[90]);
  message += seperator;
  message += String(data.lidar[180]);
  message += seperator;
  message += String(data.lidar[270]);
  message += seperator;

  message += String(data.speed);
  message += seperator;
  message += String(data.encoderPosition);
  message += seperator;  

  message += String(data.imuCalib);
  message += seperator;
  message += String(data.gyroCalib);
  message += seperator;
  message += String(data.accelCalib);
  message += seperator;
  message += String(data.magCalib);
  message += seperator;

  message += '\n';
  Serial.print(message);

}