#pragma once
#include <IDriveAlgorithm.hpp>
#include <vehiclecommand.hpp>
#include <vehicledata.hpp>
#include <config.hpp>

class hw_rev_2_SingleLidarOpenRound: public IDriveAlgorithm{

public:
    hw_rev_2_SingleLidarOpenRound(VehicleConfig cfg);
    void init() override;
    VehicleCommand drive(VehicleData vehicleData) override;

private:
    VehicleConfig _config;

};