/**
 * @brief Struct for storing data collected from sensors
 * @author DIY Labs
 */

#pragma once

#include "vec3f.hpp"
#include <cstdint>

enum SensorDataType_t{

  SENSOR_GYROSCOPE = 0,
  SENSOR_ACCELEROMETER = 1,
  SENSOR_MAGNETOMETER = 2,
  SENSOR_ORIENTATION = 3,
  SENSOR_LIDAR = 4,
  SENSOR_SPEED = 5,
  SENSOR_ENCODER = 6

};

struct SensorData{

  Vec3f orientation;
  int16_t lidar[360];
  float speed;
  SensorDataType_t sensorDataType;
  long encoderPosition;

};