#include "headers/sensors/imu.h"
// #define IMU_DEBUG
#ifdef BNO055
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29, &Wire);
uint8_t systemCali,gyroCali,accelCali,magCali;
#endif
#ifdef BNO085
Adafruit_BNO08x bno08x(-1);
#define BNO08X_ADDR 0x4A
sh2_SensorValue_t sensorValue;

#endif
Quaternion orientation;
Quaternion angularSpeed;
Quaternion bnoOrientation;
double MagX, MagY, MagZ;    // Gs

float GyroX, GyroY, GyroZ; // deg / sec
float accelBnoX, accelBnoY, accelBnoZ;   // in m/s^2
double accelBnoLinearX, accelBnoLinearY, accelBnoLinearZ;    // in m/s^2
double zGlobalAccel,yGlobalAccel,xGlobalAccel; // in m/s^2

// Compile-time or runtime fake IMU support:
// Define FAKE_IMU in build flags to default to fake mode, or set `fakeimu` at runtime.
// #ifdef FAKE_IMU
// bool fakeimu = true;
// #else
// bool fakeimu = false;
// #endif
bool fakeimu = false;

double heading;        // in radians
double zenith;        // in radians


double yaw;
double pitch;
double roll;


double bnoW;
double bnoI;
double bnoJ;
double bnoK;

void writeRegister(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(0x28); // BNO055 I2C address
    Wire.write(reg);
    Wire.write(value);
    Wire.endTransmission();
  }
#ifdef BNO085
  void bnoSetup(){
  if (!bno08x.begin_I2C()) {
    // if (!bno08x.begin_UART(&Serial1)) {  // Requires a device with > 300 byte
    // UART buffer! if (!bno08x.begin_SPI(BNO08X_CS, BNO08X_INT)) {
    Serial.println("Failed to find BNO08x chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("BNO08x found!");

  // if (bno.enableReport(SH2_ROTATION_VECTOR, 5000)){
  //   Serial.println("BNO085 rotation reporting mode configured!");
  // } 
  // else{
  //   Serial.println("BNO085 rotation reporting mode NOT configured!");
  // }
  // if (!bno08x.enableReport(SH2_ACCELEROMETER,10000)) {
  //   Serial.println("Could not enable accelerometer");
  // }
  if (!bno08x.enableReport(SH2_ROTATION_VECTOR,10000)) {
    Serial.println("Could not enable rotation");
  }
  // if (!bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED,10000)) {
  //   Serial.println("Could not enable gyro");
  // }
}
#endif
#ifdef BNO055
void bnoSetup()
  {
    // If running in fake IMU mode, skip hardware init and set sane defaults.
    if (!bno.begin())
    {
      /* There was a problem detecting the BNO055 ... check your connections */
      Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
      while (1)
        ;
    }
    bno.setExtCrystalUse(true);
    // bno.setMode(OPERATION_MODE_CONFIG);
    // bno.setMode(OPERATION_MODE_NDOF);
    bno.setMode(OPERATION_MODE_NDOF);
    delay(25); // Allow sensor to stabilize
    // writeRegister(ACC_CONFIG_REGISTER, 0x0C); // Set range to ±16g
    // Serial.println("Accelerometer set to ±16g range.");
    // Switch to ACCONLY mode
    // bno.setMode(OPERATION_MODE_NDOF_FMC_OFF);
    delay(25); // Allow sensor to stabilize

    imu::Quaternion quat = bno.getQuat();
    orientation.w = quat.w();
    orientation.i = quat.x();
    orientation.j = quat.y();
    orientation.k = quat.z();

    Serial.println("Orientation Initialized");
  
  }
#endif
  

  double degreesToRadians(double angle)
  {
    return angle / 180 * PI;
  }
  double radiansToDegrees(double angle)
  {
    return angle * 180 / PI;
  }
  
  double angleBetweenAxis(Quaternion newOrientation, String dir)
  {
    Quaternion axis;//bruh
    if      (dir == "X+"){axis.setQuat(0, 1, 0, 0);}//shit code
    else if (dir == "X-"){axis.setQuat(0, -1, 0, 0);}
    else if (dir == "Y+"){axis.setQuat(0, 0, 1, 0);}
    else if (dir == "Y-"){axis.setQuat(0, 0, -1, 0);}
    else if (dir == "Z+"){axis.setQuat(0, 0, 0, 1);}
    else if (dir == "Z-"){axis.setQuat(0, 0, 0, -1);}
  
    Quaternion copy = newOrientation;
    newOrientation.mult(axis);
    newOrientation.mult(copy.inverse());
    // angle between vectors formula:
    return radiansToDegrees(acos(newOrientation.k / newOrientation.getLength()));
    // return orientation
  }
  
  
  double axisComponent(Quaternion newOrientation, double x, double y, double z, String dir ){
    newOrientation.normalize();

    Quaternion quatVector(0, x, y, z);
    Quaternion copy = newOrientation;
    newOrientation.mult(quatVector);
    newOrientation.mult(copy.inverse());
    //orientation is now a vector normalized from the inputted vector in the global x y & z directions
    if      (dir == "X+"){return newOrientation.i;}
    else if (dir == "X-"){return -newOrientation.i;}
    else if (dir == "Y+"){return newOrientation.j;}
    else if (dir == "Y-"){return -newOrientation.j;}
    else if (dir == "Z+"){return newOrientation.k;}
    else if (dir == "Z-"){return -newOrientation.k;}
    return 0;
  }
  double verticalAcceleration(Quaternion newOrientation, double accelX, double accelY, double accelZ)
  {
    newOrientation.normalize();
    Quaternion accel(0, accelX, accelY, accelZ);
    Quaternion copy = newOrientation;
    newOrientation.mult(accel);
    newOrientation.mult(copy.inverse());
    // return i if x direction is facing top, j if y direction is facing top, and k if z direction is facing top
    // the values may be reversed if the direction is flipped
    return newOrientation.k;
  }
  
  Quaternion getQuat(){
    return bnoOrientation;
  }

  #ifdef BNO055
  void IMUdata(int deltaT)
  {
    // Possible vector values can be:
    // - VECTOR_ACCELEROMETER - m/s^2
    // - VECTOR_MAGNETOMETER  - uT
    // - VECTOR_GYROSCOPE     - rad/s
    // - VECTOR_EULER         - degrees
    // - VECTOR_LINEARACCEL   - m/s^2
    // - VECTOR_GRAVITY       - m/s^2
    // If fakeimu is enabled, generate synthetic (but plausible) IMU values.

    // imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    // imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    // imu::Vector<3> linAccel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);

    // imu::Vector<3> mag = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
  
    // imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    imu::Quaternion quat = bno.getQuat();
    /* Display the doubleing point data */
    // yaw += gyro.x()*deltaT/1000000;
    // pitch += gyro.y()*deltaT/1000000;//perpindicular to the ground
    // roll += gyro.z()*deltaT/1000000;
    // orientation.gyro(degreesToRadians(yaw),degreesToRadians(pitch),degreesToRadians(roll));
    // Quaternion orientation(quat.w(),quat.x(),quat.y(),quat.z());

    // angularSpeed.gyro(degreesToRadians(gyro.x() * deltaT / 1000000),
    //                   degreesToRadians(gyro.y() * deltaT / 1000000),
    //                   degreesToRadians(gyro.z() * deltaT / 1000000));
    //orientation.mult(angularSpeed);
    // imu::Quaternion quat = bno.getQuat();
    bnoW = quat.w();
    bnoI = quat.x();
    bnoJ = quat.y();
    bnoK = quat.z();
    // GyroY = gyro.y();

    bnoOrientation.setQuat(quat.w(),quat.x(),quat.y(),quat.z());

    zenith = angleBetweenAxis(bnoOrientation,"Y+");
    zGlobalAccel = axisComponent(bnoOrientation, accel.x(), accel.y(), accel.z(), "Z+");
    //xGlobalAccel = axisComponent(bnoOrientation, accel.x(), accel.y(), accel.z(), "X+");
    //yGlobalAccel = axisComponent(bnoOrientation, accel.x(), accel.y(), accel.z(), "Y+");
  
    //trueAngle = magAngle(orientation, mag.x(), mag.y(), mag.z());

    // MagX = mag.x();
    // MagY = mag.y();
    // MagZ = mag.z();

    accelBnoX = accel.x();
    accelBnoY = accel.y();//put negative side if servos are on top
    // accelBnoZ = accel.z();
    // accelBnoLinearX = linAccel.x();
    // accelBnoLinearY = linAccel.y();
    // accelBnoLinearZ = linAccel.z();
    bno.getCalibration(&systemCali,&gyroCali,&accelCali,&magCali);
        #ifdef IMU_DEBUG
        Serial.print(">");
        Serial.print("DeltaT: ");
        Serial.print(deltaT);
        Serial.print(", ");
        Serial.print("Accel: ");
        Serial.print(accelBnoY, DATAPRECISION);
        Serial.print(", ");
        Serial.print("LinAccel: ");
        Serial.print(accelBnoLinearY, DATAPRECISION);
        Serial.print(", ");
        Serial.print("GlobalAccel: ");
        Serial.print(zGlobalAccel, DATAPRECISION);
        Serial.print(", ");
        Serial.print("zenith: ");
        Serial.print(zenith, DATAPRECISION);
        Serial.print(", ");
        Serial.print("Roll rate: ");
        Serial.print(GyroY, DATAPRECISION);
        Serial.print(", ");
        Serial.println();
        #endif
  }
  #endif
  #ifdef BNO085

  void IMUdata(int deltaT){
  bool accelUpdated = false;
  bool gyroUpdated = false;
  bool rotUpdated   = false;


  if (bno08x.getSensorEvent(&sensorValue)) {

    switch (sensorValue.sensorId) {

    case SH2_ACCELEROMETER:
      if (accelUpdated){
        break;
      }
      accelBnoX = sensorValue.un.accelerometer.x;
      accelBnoY = sensorValue.un.accelerometer.y;
      accelBnoZ = sensorValue.un.accelerometer.z;
      accelUpdated = true;

      break;
 
    case SH2_GYROSCOPE_CALIBRATED:
      if (gyroUpdated){
        break;
      }
      GyroY = sensorValue.un.gyroscope.y;
      gyroUpdated = true;
      break;
    case SH2_ROTATION_VECTOR:
      // if (rotUpdated){
      //   break;
      // }
        bnoW = sensorValue.un.rotationVector.real;
        bnoI = sensorValue.un.rotationVector.i;
        bnoJ = sensorValue.un.rotationVector.j;
        bnoK = sensorValue.un.rotationVector.k;
        bnoOrientation.setQuat(bnoW,bnoI,bnoJ,bnoK);
        uint8_t accuracyThing;
        // 
        // accelBnoLinearY = bno.getLinAccelY() * pow(2,8*-1);

        // GyroY = bno.getGyroY() * pow(2,9*-1);
        // bno.getAccelAccuracy();

        // double quatRadianAccuracy = bno.getQuatRadianAccuracy();
        zGlobalAccel = axisComponent(bnoOrientation, accelBnoX, accelBnoY, accelBnoZ, "Z+");
        // writeZGlobalAccel(newZGlobalAccel);

        zenith = angleBetweenAxis(bnoOrientation,"Y+");
        rotUpdated = true;
        break;
   
  //  if (accelUpdated && gyroUpdated && rotUpdated) {
  //   break;
  //   }
  }

  }
}

  #endif