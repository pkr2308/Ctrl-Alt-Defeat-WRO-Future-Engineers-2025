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
  SENSOR_IMU = 3,
  SENSOR_ORIENTATION = 4,
  SENSOR_LIDAR = 5,
  SENSOR_SPEED = 6,
  SENSOR_ENCODER = 7

};

struct SensorData{

  Vec3f orientation;
  Vec3f acceleration;
  Vec3f angularVelocity;
  int16_t lidar[360];
  int16_t speed;
  SensorDataType_t sensorDataType;
  long encoderPosition;

};