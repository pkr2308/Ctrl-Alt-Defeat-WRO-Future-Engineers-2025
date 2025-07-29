/**
 * @file configuration.hpp
 * @brief Defines various structs for holding configuration data
 * @author DIY Labs
 */

 #pragma once

#include <cstdint>

class PowertrainDriver;

/**
 * @brief Phsyical pin numbers for peripherals
 */
struct PinConfig{

  uint8_t motorDriverPWM = 0;
  uint8_t motorDriverDirA = 0;
  uint8_t motorDriverDirB = 0; 
  uint8_t motorDriverStandby = 0;

  uint8_t steeringServo = 0;
  
  uint8_t nrfCS = 0;
  uint8_t nrfCE = 0;

  uint8_t i2c0SDA = 0;
  uint8_t i2c0SCL = 0;
  uint8_t i2c1SDA = 0;
  uint8_t i2c1SCL = 0;

  uint8_t spi0MOSI = 0;
  uint8_t spi0MISO = 0;
  uint8_t spi0SCK = 0;  

  uint8_t uart1RX = 0;
  uint8_t uart1TX = 0;

};

/**
 * @brief One Config Struct To Rule Them All
 */
struct VehicleConfig{

  PinConfig pinConfig;

};