#pragma once

#include <ILogger.hpp>
#include <config.hpp>
#include <vehiclecommand.hpp>
#include <vehicledata.hpp>

enum ParkState{

  PARK_STRAIGHT_UNTIL_INITIAL_TURN,
  PARK_INITIAL_TURN,

};

struct ParkConfig{

  bool clockwise;
  
  uint16_t startInitialTurnDist;
  uint16_t baseSpeed;

};

class hw_rev_2_Park{

public:
  hw_rev_2_Park(VehicleConfig cfg);

  void init(ILogger* logger, bool cw);
  VehicleCommand drive(VehicleData data);
  bool isDirectControl() {return true;}

private:
  ParkState _state;
  VehicleConfig _config;
  ParkConfig _parkConfig;
  ILogger *_logger;
  VehicleData _data;
  VehicleCommand _cmd;

  ParkConfig PARK_GET_CONFIG_CCW();
  void _straightUntilInitialTurn();

};