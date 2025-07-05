/*
  SensorManager.h
  Manages sensor data collection and processing

  Written by DIY Labs

*/

#pragma once

#include <vector>
#include <Adafruit_Sensor.h>


class Sensor; // Forward declaration of Sensor class


/**
 * @brief Manages sensor initialization, and collecting data
 */
class SensorManager{

public:
  SensorManager();

  /**
   * @brief Add a Sensor to the manager's list.
   * @param sensor Pointer to the Sensor object to be added.
   */
  void addSensor(Sensor* sensor);

  /**
   * @brief Initializes all sensors in list
   * @return True if all sensors are initialized successfully, false otherwise.
   */
  bool initializeSensors();

  /**
   * @brief Updates all sensors and collects their data.
   * @return Vector containing sensor events from all sensors.
   */

  std::vector<sensors_event_t> updateSensors();

private:
  std::vector<Sensor*> _sensors; 

};