/*

  Sensor.h
  Defines base class for sensors managed by SensorManager.
  Sensor drivers must inherit from this class.

  Written by DIY Labs

*/

#pragma once

#include <Adafruit_Sensor.h>
#include <vector>

/**
 * @brief Base class for sensors managed by SensorManager. Sensor drivers must inherit from this class.
 */
class Sensor{

  public:

  /**
   * @brief Default destructor for Sensor class.
   */
  virtual ~Sensor() = default;

  /**
   * @brief Initializes sensor.
   * @return True if initialization is successful, false otherwise.
   */
  virtual bool initialize() = 0;

  /**
   * @brief Updates sensor data.
   * @return Vector containing sensor events.
   * 
   */
  virtual std::vector<sensors_event_t> update() = 0;

  /**
   * @brief Returns the type of the sensor.
   * @return A string representing the sensor type.
   */
  virtual const char* getType() const = 0;

};