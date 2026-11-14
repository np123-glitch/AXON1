#pragma once
struct ImuSample { float acc[3]; float gyro[3]; };   // g, rad/s (bias removed)

class Imu {
public:
  bool begin();
  bool read(ImuSample& s);
  void calibrateGyro();
};