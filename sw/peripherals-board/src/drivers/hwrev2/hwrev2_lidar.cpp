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

  data.lidar[0] = 0;
  data.lidar[90] = 0;
  data.lidar[180] = 0;
  data.lidar[270] = 0;

  if(!_lidar->getData(data.lidar[0], _config.addressConfig.frontLidarAddr)){
    _logger->sendMessage("hw_rev_2_lidar::update()", _logger->ERROR, "Data read from front LiDAR returned false");
  }
  else{
    _logger->sendMessage("hw_rev_2_lidar::update()", _logger->INFO, "Data read from front LiDAR returned true");
  }

  if(!_lidar->getData(data.lidar[90], _config.addressConfig.rightLidarAddr)){
    _logger->sendMessage("hw_rev_2_lidar::update()", _logger->ERROR, "Data read from right LiDAR returned false");
  }
  else{
    _logger->sendMessage("hw_rev_2_lidar::update()", _logger->INFO, "Data read from right LiDAR returned true");
  }

  if(!_lidar->getData(data.lidar[180], _config.addressConfig.backLidarAddr)){
    _logger->sendMessage("hw_rev_2_lidar::update()", _logger->ERROR, "Data read from back LiDAR returned false");
  }
  else{
    _logger->sendMessage("hw_rev_2_lidar::update()", _logger->INFO, "Data read from back LiDAR returned true");
  }

  if(!_lidar->getData(data.lidar[270], _config.addressConfig.leftLidarAddr)){
    _logger->sendMessage("hw_rev_2_lidar::update()", _logger->ERROR, "Data read from left LiDAR returned false");
  }
  else{
    _logger->sendMessage("hw_rev_2_lidar::update()", _logger->INFO, "Data read from left LiDAR returned true");
  }

  
  dataVector.push_back(data);

  return dataVector;
 
}

String hw_rev_2_lidar::getSensorName(){
  return "TFLuna Combined Driver";
}