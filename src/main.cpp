// AXON1 — MPU6500 orientation streamer for the 3D rocket viewer
// ---------------------------------------------------------------
// * Reads the gyro + accelerometer at 200 Hz
// * Fuses them with a Madgwick filter (6-axis, no magnetometer)
// * Streams the orientation quaternion at 50 Hz:   Q,w,x,y,z
// * Lines starting with '#' are status messages
//
// Mounting convention: IMU +Y axis points toward the rocket's NOSE.
//
// Serial commands (sent by the viewer's buttons, or type them in a monitor):
//   r  = reset orientation from gravity (also zeroes the spin/heading drift)
//   c  = recalibrate the gyro (keep the rocket perfectly still for ~2 s)

#include <Arduino.h>
#include <Wire.h>

// ---- Wiring: use the same pins your I2C scanner worked with ----
const int SDA_PIN = 21;
const int SCL_PIN = 22;

const uint8_t MPU = 0x68;

// ±16 g and ±2000 °/s ranges
const float ACC_LSB_PER_G   = 2048.0f;
const float GYRO_LSB_PER_DPS = 16.4f;
const float DEG2RAD = 0.01745329252f;

const uint32_t LOOP_US  = 5000;  // 200 Hz filter update
const int      PRINT_EVERY = 4;  // 200 / 4 = 50 Hz output

// Madgwick gain: higher = trusts the accelerometer more (faster correction,
// more jitter); lower = trusts the gyro more (smoother, slower correction).
float beta = 0.1f;

float q0 = 1, q1 = 0, q2 = 0, q3 = 0;   // body -> earth quaternion (earth Z = up)
float gBias[3] = {0, 0, 0};             // gyro bias in °/s

// ---------------------------------------------------------------- IMU I/O
void writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

uint8_t readReg(uint8_t reg) {
  Wire.beginTransmission(MPU);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return 0xFF;
  if (Wire.requestFrom(MPU, (uint8_t)1) != 1) return 0xFF;
  return Wire.read();
}

// acc in g, gyro in °/s (raw, bias NOT removed). Returns false on I2C error.
bool readIMU(float acc[3], float gyro[3]) {
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);                       // ACCEL_XOUT_H
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(MPU, (uint8_t)14) != 14) return false;

  int16_t raw[7];
  for (int i = 0; i < 7; i++) {
    uint8_t hi = Wire.read();
    uint8_t lo = Wire.read();
    raw[i] = (int16_t)((hi << 8) | lo);
  }
  for (int i = 0; i < 3; i++) {
    acc[i]  = raw[i]     / ACC_LSB_PER_G;
    gyro[i] = raw[i + 4] / GYRO_LSB_PER_DPS;   // raw[3] is temperature
  }
  return true;
}

// ---------------------------------------------------------------- calibration
void calibrateGyro() {
  Serial.println("# Calibrating gyro - keep the rocket still...");
  const int N = 1000;
  double sum[3] = {0, 0, 0};
  int good = 0;
  float a[3], g[3];
  for (int i = 0; i < N; i++) {
    if (readIMU(a, g)) {
      for (int k = 0; k < 3; k++) sum[k] += g[k];
      good++;
    }
    delay(2);
  }
  if (good > 0) for (int k = 0; k < 3; k++) gBias[k] = sum[k] / good;
  Serial.printf("# Gyro bias (deg/s): %.2f %.2f %.2f\n", gBias[0], gBias[1], gBias[2]);
}

// Start the filter at the orientation gravity says we're in, so it doesn't
// have to slowly converge from "lying flat". Heading (spin) is set to zero.
void initFromAccel() {
  float a[3], g[3];
  float sum[3] = {0, 0, 0};
  int good = 0;
  for (int i = 0; i < 50; i++) {
    if (readIMU(a, g)) { for (int k = 0; k < 3; k++) sum[k] += a[k]; good++; }
    delay(2);
  }
  if (good == 0) return;
  float ax = sum[0], ay = sum[1], az = sum[2];
  float n = sqrtf(ax * ax + ay * ay + az * az);
  if (n < 1e-6f) return;
  ax /= n; ay /= n; az /= n;

  // Shortest rotation taking the measured "up" (accel direction) onto earth +Z
  if (az < -0.9999f) {          // exactly upside down
    q0 = 0; q1 = 1; q2 = 0; q3 = 0;
  } else {
    q0 = 1.0f + az; q1 = ay; q2 = -ax; q3 = 0;
    float m = sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 /= m; q1 /= m; q2 /= m; q3 /= m;
  }
  Serial.println("# Orientation reset from gravity");
}

// ---------------------------------------------------------------- Madgwick (IMU)
// gx,gy,gz in rad/s; ax,ay,az in any unit; dt in seconds
void madgwickUpdate(float gx, float gy, float gz, float ax, float ay, float az, float dt) {
  float qDot0 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
  float qDot1 = 0.5f * ( q0 * gx + q2 * gz - q3 * gy);
  float qDot2 = 0.5f * ( q0 * gy - q1 * gz + q3 * gx);
  float qDot3 = 0.5f * ( q0 * gz + q1 * gy - q2 * gx);

  float an = sqrtf(ax * ax + ay * ay + az * az);
  // Only correct with the accelerometer when it's measuring roughly 1 g
  // (ignores it during hard shakes / motor thrust)
  if (an > 0.5f && an < 1.5f) {
    ax /= an; ay /= an; az /= an;

    float _2q0 = 2.0f * q0, _2q1 = 2.0f * q1, _2q2 = 2.0f * q2, _2q3 = 2.0f * q3;
    float _4q0 = 4.0f * q0, _4q1 = 4.0f * q1, _4q2 = 4.0f * q2;
    float _8q1 = 8.0f * q1, _8q2 = 8.0f * q2;
    float q0q0 = q0 * q0, q1q1 = q1 * q1, q2q2 = q2 * q2, q3q3 = q3 * q3;

    float s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
    float s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1
               + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
    float s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2
               + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
    float s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;

    float sn = sqrtf(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
    if (sn > 1e-9f) {
      qDot0 -= beta * s0 / sn;
      qDot1 -= beta * s1 / sn;
      qDot2 -= beta * s2 / sn;
      qDot3 -= beta * s3 / sn;
    }
  }

  q0 += qDot0 * dt; q1 += qDot1 * dt; q2 += qDot2 * dt; q3 += qDot3 * dt;
  float qn = sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 /= qn; q1 /= qn; q2 /= qn; q3 /= qn;
}

// ---------------------------------------------------------------- setup / loop
void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);
  delay(200);

  uint8_t id = readReg(0x75);
  Serial.printf("# WHO_AM_I = 0x%02X\n", id);
  while (id != 0x70 && id != 0x71 && id != 0x73) {
    Serial.println("# IMU not found - check wiring / SDA_PIN / SCL_PIN");
    delay(1000);
    id = readReg(0x75);
  }

  writeReg(0x6B, 0x80);  delay(100);   // reset the chip
  writeReg(0x6B, 0x01);  delay(50);    // wake up, gyro PLL clock
  writeReg(0x19, 0x00);                // sample-rate divider = 0
  writeReg(0x1A, 0x03);                // gyro low-pass ~41 Hz
  writeReg(0x1B, 0x18);                // gyro ±2000 °/s
  writeReg(0x1C, 0x18);                // accel ±16 g
  writeReg(0x1D, 0x03);                // accel low-pass ~41 Hz
  delay(50);

  calibrateGyro();
  initFromAccel();
  Serial.println("# Streaming");
}

void loop() {
  // Commands from the viewer
  while (Serial.available()) {
    char c = Serial.read();
    if (c == 'r') initFromAccel();
    if (c == 'c') { calibrateGyro(); initFromAccel(); }
  }

  static uint32_t last = micros();
  uint32_t now = micros();
  if (now - last < LOOP_US) return;
  float dt = (now - last) * 1e-6f;
  last = now;
  if (dt > 0.05f) dt = 0.05f;          // after a pause (e.g. calibration)

  float a[3], g[3];
  if (!readIMU(a, g)) return;

  madgwickUpdate((g[0] - gBias[0]) * DEG2RAD,
                 (g[1] - gBias[1]) * DEG2RAD,
                 (g[2] - gBias[2]) * DEG2RAD,
                 a[0], a[1], a[2], dt);

  static int n = 0;
  if (++n >= PRINT_EVERY) {
    n = 0;
    Serial.printf("Q,%.4f,%.4f,%.4f,%.4f\n", q0, q1, q2, q3);
  }
}