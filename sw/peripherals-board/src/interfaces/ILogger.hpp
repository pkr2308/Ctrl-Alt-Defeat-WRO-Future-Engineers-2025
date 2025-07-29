#pragma once

#include <Arduino.h>

class ILogger{
public:  

  virtual ~ILogger() = default;

  virtual void init() = 0;
  virtual void sendMessage(String sender, String message);

};