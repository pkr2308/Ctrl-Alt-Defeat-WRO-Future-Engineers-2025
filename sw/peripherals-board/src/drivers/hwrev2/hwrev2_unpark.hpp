#pragma once

#include <IDriveAlgorithm.hpp>
#include <config.hpp>

class hw_rev_2_UnparkAlgorithm: public IDriveAlgorithm{

public:
  hw_rev_2_UnparkAlgorithm(VehicleConfig cfg);
  void init(ILogger *logger) override;
  VehicleCommand drive(VehicleData data) override;
  bool isDirectControl() override {return true;}

private:
  VehicleConfig _config;
  ILogger *_logger;

};