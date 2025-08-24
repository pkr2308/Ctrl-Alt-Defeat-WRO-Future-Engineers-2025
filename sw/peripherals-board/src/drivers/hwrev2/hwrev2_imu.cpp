/**
 * @brief Implementation of hwrev2 IMU driver
 * @author DIY Labs
 */

#include "hwrev2_imu.hpp"
#include <Adafruit_BNO055.h>

/**
 * @brief Constructor for BNO055 driver
 * @param cfg Vehicle configuration struct
 */
hw_rev_2_imu::hw_rev_2_imu(VehicleConfig cfg){
  
  _config = cfg;

}

/**
 * @brief Initializes BNO055
 * @todo Send recorded calibration data to BNO055
 */
status_t hw_rev_2_imu::init(ILogger *logger){

  _logger = logger;

  _bno = new Adafruit_BNO055(12345, _config.addressConfig.bnoAddr, &Wire);
  bool status = _bno->begin();

  if(!status){
    _logger->sendMessage("hw_rev_2_imu::init", _logger->ERROR, "Failed to initialize BNO055");
    return STATUS_FAULT;
  }
  else{
    _logger->sendMessage("hw_rev_2_imu::init", _logger->INFO, "Successfully initialized BNO055");
    return STATUS_HEALTHY;
  }

}

std::vector<SensorData> hw_rev_2_imu::update(){

  std::vector<SensorData> dataVector;
  SensorData data;
  SensorData calibration;
  data.sensorDataType = SENSOR_IMU;
  calibration.sensorDataType = SENSOR_IMU_CALIBRATION;

  sensors_event_t bnoData;
  _bno->getEvent(&bnoData);

  data.orientation.x = bnoData.orientation.x;
  data.orientation.y = bnoData.orientation.y;
  data.orientation.z = bnoData.orientation.z;
  data.angularVelocity.x = bnoData.gyro.x;
  data.angularVelocity.y = bnoData.gyro.y;
  data.angularVelocity.z = bnoData.gyro.z;
  data.acceleration.x = bnoData.acceleration.x;
  data.acceleration.y = bnoData.acceleration.y;
  data.acceleration.z = bnoData.acceleration.z;

  uint8_t sysCal, gyroCal, accelCal, magCal;
  _bno->getCalibration(&sysCal, &gyroCal, &accelCal, &magCal);

  calibration.gyroCalib = gyroCal;
  calibration.accelCalib = accelCal;
  calibration.magCalib = magCal;
  calibration.sysCalib = sysCal;

  dataVector.push_back(data);
  dataVector.push_back(calibration);

  return dataVector;
 
}

String hw_rev_2_imu::getSensorName(){
  return "BNO055 Driver";
}