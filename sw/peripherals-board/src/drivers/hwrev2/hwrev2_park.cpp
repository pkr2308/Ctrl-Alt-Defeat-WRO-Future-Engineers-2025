#include <hwrev2_park.hpp>

hw_rev_2_Park::hw_rev_2_Park(VehicleConfig cfg){

  _config = cfg;

}

void hw_rev_2_Park::init(ILogger* logger, bool cw){
  
  _state = PARK_STRAIGHT_UNTIL_INITIAL_TURN;
  
  if(cw){

  }
  else{
    _parkConfig = PARK_GET_CONFIG_CCW();
  }

}

VehicleCommand hw_rev_2_Park::drive(VehicleData vehicleData){

  _data = vehicleData;

  switch(_state){
    
    case PARK_STRAIGHT_UNTIL_INITIAL_TURN:
      _straightUntilInitialTurn();

  }

  return _cmd;

}

void hw_rev_2_Park::_straightUntilInitialTurn(){

  _cmd.targetSpeed = 200;
  _cmd.targetYaw = 90;

}

ParkConfig hw_rev_2_Park::PARK_GET_CONFIG_CCW(){

  ParkConfig parkCfg;

  parkCfg.clockwise = false;
  parkCfg.startInitialTurnDist = 50;
  parkCfg.baseSpeed = 200;

  return parkCfg;

}