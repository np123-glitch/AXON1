#pragma once
#include <stdint.h>
// I2C
constexpr int SDA_PIN = 21, SCL_PIN = 22;
// Servos
constexpr int SERVO_A_PIN = 25, SERVO_B_PIN = 26;
constexpr float SERVO_A_TRIM = 0.0f, SERVO_B_TRIM = 0.0f;  // degrees, set on the bench
constexpr float GIMBAL_LIMIT_DEG = 8.0f;                   // max deflection
// Timing
constexpr uint32_t LOOP_HZ = 200;
constexpr uint32_t LOOP_US = 1000000 / LOOP_HZ;
constexpr float    DT      = 1.0f / LOOP_HZ;