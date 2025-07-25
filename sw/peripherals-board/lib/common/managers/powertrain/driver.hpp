/**
 * @file driver.hpp
 * @brief Defines class for driver for powertrain manager
 * @author DIY Labs
 */

 #include <configuration.hpp>
 #include <cstdint>

class PowertrainDriver{

public:
  virtual void init(VehicleConfig cfg) = 0;
  virtual void commandMotor(uint16_t speed, bool dir, uint16_t absMaxCommand) = 0;
  virtual void commandSteering(uint16_t cmdDegrees) = 0;

};