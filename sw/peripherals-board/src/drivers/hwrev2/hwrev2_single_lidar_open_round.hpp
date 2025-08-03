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
    bool isDirectControl() override { return true; }

private:
    VehicleConfig _config;

    // About driving
    int dir = 1;
    uint16_t speed = 275;  // Motor PWM speed 
    int lastEncoded = 0; 
    long encoderValue = 0;
    float distance = 0.0;

    // About turning
    int turns = 0;
    bool turning = false;
    int turnDir = 1;           // 1 for clockwise, -1 for counterclockwise
    
    int pos = 90;    // variable to store the servo position  

    // About IMU
    float yaw;
    uint8_t targetYaw = 0;
    int startYaw = 0;

    unsigned long startMillis;
    unsigned long currentMillis;
    const unsigned long period = 200;


    // About Lidar
    int threshold = 60;
    int16_t lidarDist;
    int16_t startDist = 50;
    int stopDist = 5;

    // About Gyro straight follower
    int correction = 0;
    float error = 0;
    float totalError = 0;             // Used for integral control

    bool completed = false; // Indicates if the 3 rounds are completed
};