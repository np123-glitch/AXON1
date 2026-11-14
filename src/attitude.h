#pragma once
#include "imu.h"

class Attitude {
public:
  void initFromAccel(const ImuSample& s);
  void update(const ImuSample& s, float dt);
  float tiltX() const;   // nose toward body +X, degrees
  float tiltZ() const;   // nose toward body +Z, degrees
  float q[4] = {1, 0, 0, 0};
};