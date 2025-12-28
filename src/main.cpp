/*
 float ax = (rawX - X_OFFSET) * X_SCALE / G_LSB;
float ay = (rawY - Y_OFFSET) * Y_SCALE / G_LSB;
float az = (rawZ - Z_OFFSET) * Z_SCALE / G_LSB;

Offsets:
X_OFFSET = -4.33
Y_OFFSET = 4.44
Z_OFFSET = 8.03

Scale factors:
X_SCALE = 0.994841
Y_SCALE = 0.991441
Z_SCALE = 1.034657

*/

#include <Wire.h>
#include <HardwareSerial.h>

#define ADXL343_ADDR 0x53

#define REG_POWER_CTL   0x2D
#define REG_DATA_FORMAT 0x31
#define REG_DATAX0      0x32

#define SAMPLES 200
#define G_LSB 256.0f   // ~256 LSB = 1g for ADXL343 @ ±2g

struct Vec3 {
  float x, y, z;
};

Vec3 meas[6];   // +X, -X, +Y, -Y, +Z, -Z
Vec3 offset;
Vec3 scale;

void writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(ADXL343_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

int16_t readAxis(uint8_t reg) {
  Wire.beginTransmission(ADXL343_ADDR);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(ADXL343_ADDR, 2);
  return (int16_t)(Wire.read() | (Wire.read() << 8));
}

Vec3 readAverage() {
  long sx = 0, sy = 0, sz = 0;

  for (int i = 0; i < SAMPLES; i++) {
    sx += readAxis(REG_DATAX0);
    sy += readAxis(REG_DATAX0 + 2);
    sz += readAxis(REG_DATAX0 + 4);
    delay(5);
  }

  return {
    sx / (float)SAMPLES,
    sy / (float)SAMPLES,
    sz / (float)SAMPLES
  };
}

void waitForEnter() {
  while (!Serial.available());
  while (Serial.available()) Serial.read();
}

void prompt(const char* msg) {
  Serial.println();
  Serial.println(msg);
  Serial.println("Press ENTER when ready...");
  waitForEnter();
}

void setup() {
  Serial.begin(115200);
  Wire.begin(5, 22);

  writeReg(REG_POWER_CTL, 0x08);   // Measure mode
  writeReg(REG_DATA_FORMAT, 0x00); // ±2g

  Serial.println("=== ADXL343 3D Calibration ===");

  prompt("Place sensor with +X pointing UP");
  meas[0] = readAverage();

  prompt("Place sensor with -X pointing UP");
  meas[1] = readAverage();

  prompt("Place sensor with +Y pointing UP");
  meas[2] = readAverage();

  prompt("Place sensor with -Y pointing UP");
  meas[3] = readAverage();

  prompt("Place sensor with +Z pointing UP");
  meas[4] = readAverage();

  prompt("Place sensor with -Z pointing UP");
  meas[5] = readAverage();

  // Offsets
  offset.x = (meas[0].x + meas[1].x) / 2.0f;
  offset.y = (meas[2].y + meas[3].y) / 2.0f;
  offset.z = (meas[4].z + meas[5].z) / 2.0f;

  // Scale factors
  scale.x = (2.0f * G_LSB) / (meas[0].x - meas[1].x);
  scale.y = (2.0f * G_LSB) / (meas[2].y - meas[3].y);
  scale.z = (2.0f * G_LSB) / (meas[4].z - meas[5].z);

  Serial.println("\n=== CALIBRATION RESULTS ===");

  Serial.println("\nOffsets:");
  Serial.print("X_OFFSET = "); Serial.println(offset.x);
  Serial.print("Y_OFFSET = "); Serial.println(offset.y);
  Serial.print("Z_OFFSET = "); Serial.println(offset.z);

  Serial.println("\nScale factors:");
  Serial.print("X_SCALE = "); Serial.println(scale.x, 6);
  Serial.print("Y_SCALE = "); Serial.println(scale.y, 6);
  Serial.print("Z_SCALE = "); Serial.println(scale.z, 6);

  Serial.println("\nCopy these into your main program.");
}

void loop() {
  // Nothing – calibration done
}