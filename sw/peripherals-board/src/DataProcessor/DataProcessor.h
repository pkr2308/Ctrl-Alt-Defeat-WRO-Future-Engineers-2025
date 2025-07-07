/*
  DataProcessor.h
  Utility class to processes data from IMU into vehicle state

  Written by DIY Labs

*/
#pragma once
#include "VehicleState.h"
#include <vector>
#include <Adafruit_Sensor.h>

class DataProcessor{

public:
    DataProcessor();
    ~DataProcessor();

    /**
     * @brief Processes gyroscope and accelerometer data into VehicleState
     * @param sensorData Vector of sensor events, output from SensorManager
     * @return VehicleState containing orientation and velocity
     */
    VehicleState processSensorData(std::vector <sensors_event_t> sensorData);

  private:
  uint16_t _currentMillis = 0, _prevMillis=  0;

  VehicleState _state;

};