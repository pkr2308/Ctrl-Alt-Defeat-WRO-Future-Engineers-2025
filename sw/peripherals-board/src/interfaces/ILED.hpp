/**
 * @brief Interface for RGB LED
 * @author DIY Labs
 */

#pragma once

#include <ILogger.hpp>

struct RGBColor{

  uint8_t red;
  uint8_t green;
  uint8_t blue;

};

struct RGBBlink{

  RGBColor onColor;
  RGBColor offColor;

  uint32_t onMs;
  uint32_t offMs;

};

class ILED{
public:  

  virtual ~ILED() = default;

  virtual void init(ILogger *logger) = 0;
  virtual void setStaticColor(RGBColor col) = 0;
  virtual void limitBrightness(uint8_t lim);

};