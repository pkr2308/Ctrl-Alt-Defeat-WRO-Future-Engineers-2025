#pragma once

#include <IRemoteCommunication.hpp>
#include <config.hpp>
#include <status.hpp>
#include <vehiclecommand.hpp>
#include <vehicledata.hpp>

class hw_rev_1_RemoteCommunicationDriver{

public:
    hw_rev_1_RemoteCommunicationDriver(VehicleConfig cfg);


};

/*
#pragma once

#include <vehicledata.hpp>
#include <vehiclecommand.hpp>
#include <status.hpp>

class IRemoteCommunication{
public:  

  virtual ~IRemoteCommunication() = default;

  virtual status_t init() = 0;
  virtual VehicleCommand update(VehicleData vehicleData);

};*/