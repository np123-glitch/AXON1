#pragma once
class Tvc {
public:
  void begin();
  void center();
  void setGimbal(float aDeg, float bDeg);   // clamps to GIMBAL_LIMIT_DEG, applies trims
};