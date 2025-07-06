/*
  MPU6050.cpp
  Driver for MPU6050 IMU 
  Written by DIY Labs

*/

#define DRIVER_MPU6050_DEBUG   // Uncomment to enable debug messages

#include "MPU6050.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

DriverMPU6050::DriverMPU6050(TwoWire* wire){

  _wire = wire;
 
}

bool DriverMPU6050::initialize(){

  if(!_mpu.begin(MPU6050_I2CADDR_DEFAULT, _wire)){

    #ifdef DRIVER_MPU6050_DEBUG
      Serial.println("DriverMPU6050: Failed to initialize MPU6050");
    #endif
    return false;

  }
  else{

    #ifdef DRIVER_MPU6050_DEBUG
      Serial.println("DriverMPU6050: Successfully initialized MPU6050");
    #endif

    _mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
    _mpu.setGyroRange(MPU6050_RANGE_250_DEG);
    _mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

    return true;

  }

}

std::vector<sensors_event_t> DriverMPU6050::update(){

  std::vector<sensors_event_t> events;

  sensors_event_t accel, gyro, temp;

  _mpu.getEvent(&accel, &gyro, &temp);

  events.push_back(accel);
  events.push_back(gyro);
  events.push_back(temp);

  return events;

}

