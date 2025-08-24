#include <hwrev2_serial_communication.hpp>

hw_rev_2_SerialCommunication *commClassPtr = nullptr;

hw_rev_2_SerialCommunication::hw_rev_2_SerialCommunication(VehicleConfig cfg){

  _config = cfg;
  commClassPtr = this;

}

void hw_rev_2_SerialCommunication::init(ILogger *logger){

  _logger = logger;
  _serialTask = new SchedulerTask(_serialCallbackWrapper, 10);
  Serial.begin(115200);

}

VehicleCommand hw_rev_2_SerialCommunication::update(VehicleData data, VehicleCommand cmd){

  VehicleCommand returnCommand;

  _serialTask->update();

  returnCommand.targetSpeed = _targetSpeed;
  returnCommand.targetYaw = _targetYaw;
  returnCommand.instruction = _instruction;

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

void hw_rev_2_SerialCommunication::_serialCallback(){

  Serial.println("test");
  _instruction = NO_INSTRUCTION;
  _targetSpeed = 0;
  _targetYaw = 0;

}

void hw_rev_2_SerialCommunication::_serialCallbackWrapper(){
  commClassPtr->_serialCallback();
}