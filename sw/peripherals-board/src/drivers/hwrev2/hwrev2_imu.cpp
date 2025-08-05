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

SensorData hw_rev_2_imu::update(){

  SensorData data;
  data.sensorDataType = SENSOR_ORIENTATION;

  sensors_event_t bnoData;
  _bno->getEvent(&bnoData);

  data.orientation.x = bnoData.orientation.x;
  data.orientation.y = bnoData.orientation.y;
  data.orientation.z = bnoData.orientation.z;

  return data;
 
}