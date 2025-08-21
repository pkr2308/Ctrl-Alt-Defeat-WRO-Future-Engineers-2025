/**
 * @brief Implementation of hwrev2 lidar driver
 * @author DIY Labs
 */

#include "hwrev2_lidar.hpp"
#include <Adafruit_BNO055.h>

hw_rev_2_lidar::hw_rev_2_lidar(VehicleConfig cfg){
  
  _config = cfg;

}

void hw_rev_2_lidar::init(ILogger* logger){

  _logger = logger;
  _lidar = new TFLI2C();

}

std::vector<SensorData> hw_rev_2_lidar::update(){

  std::vector<SensorData> dataVector;
  SensorData data;
  data.sensorDataType = SENSOR_LIDAR;


  _lidar->getData(data.lidar[0], _config.addressConfig.frontLidarAddr);
  _lidar->getData(data.lidar[90], _config.addressConfig.rightLidarAddr);
  _lidar->getData(data.lidar[180], _config.addressConfig.backLidarAddr);
  _lidar->getData(data.lidar[270], _config.addressConfig.leftLidarAddr);

  dataVector.push_back(data);

  return dataVector;
 
}