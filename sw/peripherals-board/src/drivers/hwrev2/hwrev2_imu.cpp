/**
 * @brief Implementation of hwrev2 IMU driver
 * @author DIY Labs
 */

#include "hwrev2_imu.hpp"
#include <Adafruit_BNO055.h>

hw_rev_2_imu::hw_rev_2_imu(VehicleConfig cfg){
  
  _config = cfg;

}

void hw_rev_2_imu::init(){

  _bno = new Adafruit_BNO055(12345, _config.addressConfig.bnoAddr, &Wire);
  _bno->begin();

}

std::vector<SensorData> hw_rev_2_imu::update(){

  std::vector<SensorData> dataVector;
  SensorData data;
  data.sensorDataType = SENSOR_IMU;

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

  dataVector.push_back(data);

  return dataVector;
 
}