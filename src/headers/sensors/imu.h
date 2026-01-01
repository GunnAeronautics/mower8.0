#ifndef IMU_H
#define IMU_H
// #define BNO055
#define BNO085
#include <Arduino.h>
#include "../math/Quaternion.h"

#ifdef BNO055
#include <Adafruit_BNO055.h>
#endif
#ifdef BNO085
#include "SparkFun_BNO08x_Arduino_Library.h"
#endif

#include <Wire.h>
#include "../config/config.h"

extern Quaternion orientation;
extern Quaternion bnoOrientation;

extern float GyroX, GyroY, GyroZ; // deg / sec
extern float AccX, AccY, AccZ;    // Gs
extern float MagX, MagY, MagZ;    // Gs

extern double zGlobalAccel; // in m/s^2
extern double xGlobalAccel; // in m/s^2
extern double yGlobalAccel; // in m/s^2

extern float heading;        // in radians
extern float zenith;         // in radians

extern float yaw;
extern float pitch;
extern float roll;

extern float bnoW;
extern float bnoI;
extern float bnoJ;
extern float bnoK;
#ifdef BNO055
extern uint8_t systemCali, gyroCali, accelCali, magCali;
#endif
void writeRegister(uint8_t reg, uint8_t value);
void bnoSetup();
void IMUdata(int deltaT);
float verticalAcceleration(Quaternion orientation, float accelX, float accelY, float accelZ);
float axisComponent(Quaternion orientation, float x, float y, float z, String dir);
float angleBetweenAxis(Quaternion orientation, String dir);
float radiansToDegrees(float angle);
float degreesToRadians(float angle);
Quaternion getQuat();
double readZGlobalAccel();

#endif // IMU_H
