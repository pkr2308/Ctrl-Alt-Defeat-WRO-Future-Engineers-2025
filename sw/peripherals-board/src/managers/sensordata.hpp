#pragma once

#include "vec3f.hpp"
#include <cstdint>

enum SensorDataType_t{

  SENSOR_GYROSCOPE = 0,
  SENSOR_ACCELEROMETER = 1,
  SENSOR_MAGNETOMETER = 2,
  SENSOR_ORIENTATION = 3,
  SENSOR_LIDAR = 4

};

struct SensorData{

  Vec3f orientation;
  uint8_t lidar[360];
  SensorDataType_t sensorDataType;

};