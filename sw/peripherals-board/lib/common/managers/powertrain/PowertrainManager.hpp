/**
 * @file PowertrainManager.hpp
 * @brief Handles drive motor and steering servo
 * @author DIY Labs
 */

#include "driver.hpp"
#include <configuration.hpp>

class PowertrainManager{

public:
  PowertrainManager(PowertrainDriver ptd, VehicleConfig cfg);
  ~PowertrainManager() = default;

  void init();
  void drive(uint16_t speed, uint8_t steeringAngle);


private:
  PowertrainDriver _driver;
  VehicleConfig _config;

};