#include <ILogger.hpp>
#include <Arduino.h>
#include <config.hpp>


class hw_rev_1_SerialLog : public ILogger{

public:
  hw_rev_1_SerialLog(VehicleConfig cfg);
  void init() override;
  void sendMessage(String sender, String message);

private:
  VehicleConfig _config;

};