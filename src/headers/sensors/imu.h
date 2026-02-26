#define IMU_H
// #define BNO055
#define BNO085
#include <Arduino.h>
#include "../math/Quaternion.h"

#ifdef BNO055
#include <Adafruit_BNO055.h>
#endif
#ifdef BNO085
#include "Adafruit_BNO08x.h"
#endif

#include <Wire.h>
#include "../config/config.h"
#ifdef IMU_H
extern Quaternion orientation;
extern Quaternion bnoOrientation;


extern float GyroX, GyroY, GyroZ; // deg / sec

extern float accelBnoX, accelBnoY, accelBnoZ;   // in m/s^2
extern double accelBnoLinearX, accelBnoLinearY, accelBnoLinearZ;    // in m/s^2
extern double zGlobalAccel,yGlobalAccel,xGlobalAccel; // in m/s^2

extern double MagX, MagY, MagZ;    // Gs
extern double heading;        // in radians
extern double zenith;         // in radians

extern double yaw;
extern double pitch;
extern double roll;

extern double bnoW;
extern double bnoI;
extern double bnoJ;
extern double bnoK;
#ifdef BNO055
extern uint8_t systemCali, gyroCali, accelCali, magCali;
#endif
void writeRegister(uint8_t reg, uint8_t value);
void bnoSetup();
void IMUdata(int deltaT);
double verticalAcceleration(Quaternion orientation, double accelX, double accelY, double accelZ);
double axisComponent(Quaternion orientation, double x, double y, double z, String dir);
double angleBetweenAxis(Quaternion orientation, String dir);
double radiansToDegrees(double angle);
double degreesToRadians(double angle);
Quaternion getQuat();

#endif // IMU_H
