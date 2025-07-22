/*
  vector.h
  Defines 2d and 3d vector types
  Written by DIY Labs

*/
#pragma once

struct Vec2f{

  float x, y;
  Vec2f(float x = 0, float y = 0) : x(x), y(y) {}

};

struct Vec3f{

  float x, y, z;
  Vec3f(float x = 0, float y = 0, float z = 0) : x(x), y(y), z(z) {}

};