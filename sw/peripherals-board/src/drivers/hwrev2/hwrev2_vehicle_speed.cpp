/**
 * @brief Implementation of hwrev2 speed sensor driver
 * @author DIY Labs
 */

#include "hwrev2_vehicle_speed.hpp"
#include <RotaryEncoder.h>

static RotaryEncoder *_isrEncoder;
hw_rev_2_VehicleSpeed *classptr;

static void externISR(){

  classptr->_encoderISR();

}

hw_rev_2_VehicleSpeed::hw_rev_2_VehicleSpeed(VehicleConfig cfg){
  
  _config = cfg;
  _ticksPerCM = _config.constantsConfig.ticksPerCM;

}

void hw_rev_2_VehicleSpeed::init(){

  _encoder = new RotaryEncoder(_config.pinConfig.motorEncoderA, _config.pinConfig.motorEncoderB, RotaryEncoder::LatchMode::TWO03);
  _isrEncoder = _encoder;
  classptr = this;

  attachInterrupt(digitalPinToInterrupt(_config.pinConfig.motorEncoderA), externISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(_config.pinConfig.motorEncoderB), externISR, CHANGE);

}

std::vector<SensorData> hw_rev_2_VehicleSpeed::update(){

  std::vector<SensorData> dataVector;
  SensorData data;
  data.sensorDataType = SENSOR_ENCODER;
/*
  unsigned long currentTime = millis();
  _encoderPosition = _encoder->getPosition();
  float speed = 0.0f;

  unsigned long deltaTime = currentTime - lastUpdateTime;
  long deltaTicks = _encoderPosition - _prevEncoderPosition;

  if (deltaTime > 0) {
    float distanceCM = deltaTicks / (float)_ticksPerCM;
    float speedCMperS = (distanceCM * 1000.0f) / (float)deltaTime;
    speed = speedCMperS;
  }

  _prevEncoderPosition = _encoderPosition;
  lastUpdateTime = currentTime;


*/

  data.encoderPosition = -_encoder->getPosition();
  
  dataVector.push_back(data);

  return dataVector;
 
}

void hw_rev_2_VehicleSpeed::_encoderISR(){

  _encoder->tick();

}