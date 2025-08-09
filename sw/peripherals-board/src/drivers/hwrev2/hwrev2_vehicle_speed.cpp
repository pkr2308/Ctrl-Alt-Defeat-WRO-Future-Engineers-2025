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

void hw_rev_2_VehicleSpeed::init(ILogger* logger){

  _logger = logger;

  _encoder = new RotaryEncoder(_config.pinConfig.motorEncoderA, _config.pinConfig.motorEncoderB, RotaryEncoder::LatchMode::TWO03);
  _isrEncoder = _encoder;
  classptr = this;

  attachInterrupt(digitalPinToInterrupt(_config.pinConfig.motorEncoderA), externISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(_config.pinConfig.motorEncoderB), externISR, CHANGE);

}

std::vector<SensorData> hw_rev_2_VehicleSpeed::update(){

    std::vector<SensorData> dataVector;
    SensorData encPos;
    SensorData calcSpeed;
    long currentMillis = 0;
    long currentEncPos = 0;
    float speed = 0.0f;
    unsigned long deltaTime = 0;
    long deltaTicks = 0;

    encPos.sensorDataType = SENSOR_ENCODER;
    calcSpeed.sensorDataType = SENSOR_SPEED;

    currentMillis = millis();
    currentEncPos = -_encoder->getPosition(); // invert encPos since power transfer gears invert rotation

    deltaTime = currentMillis - _prevMillis;
    deltaTicks = currentEncPos - _prevEncPos;
    
    if(deltaTime > 0){
        float distanceCM = static_cast<float>(deltaTicks) / static_cast<float>(_ticksPerCM);
        speed = (distanceCM * 1000.0f) / static_cast<float>(deltaTime);
    }

    _prevEncPos = currentEncPos;
    _prevMillis = currentMillis;

    encPos.encoderPosition = currentEncPos;
    calcSpeed.speed = speed;
    dataVector.push_back(encPos);
    dataVector.push_back(calcSpeed);

    return dataVector;

}  

void hw_rev_2_VehicleSpeed::_encoderISR(){

  _encoder->tick();

}