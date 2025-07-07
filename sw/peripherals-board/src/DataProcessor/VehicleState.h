/*
  VehicleState.h
  Defines struct for vehicle state

  Written by DIY Labs

*/

#pragma once

#include <vector>

struct Vec3f{

    double x, y, z;

};

struct VehicleState{

    Vec3f orientation = {0, 0, 0};
    Vec3f velocity = {0, 0, 0};

    Vec3f rawGyro;
    Vec3f rawAccel;

};