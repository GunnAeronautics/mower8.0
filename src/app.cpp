// Sup guys this is Tony the main programmer on the ASTRA25-26 tarc team
// make sure to use camelCase for variables and functions
// use ALLCAPS and UNDER_SCORES for definitions
//
#ifdef TEST_MODE
#include "../test/test_mocks.h"
#include <algorithm>
#else
#include <Arduino.h>
#include <math.h>
// #include <SimpleKalmanFilter.h>
// SPIFFS removed in favor of SD logging
// Configuration and constants
#include "headers/config/config.h"

// Homebrew libraries:
// #include "headers/math/KalmanVelocityEstimator.h"
#include "headers/sensors/adxl.h"
#include "headers/sensors/bmpxxx.h"
#include "headers/sensors/imu.h"
#include "headers/utils/sdcard.h"
#include "headers/utils/servos.h"
#include "headers/utils/led.h"

#include "esp_partition.h"
#endif

// Last tuned before 2nd launch 3/22/25 pls don't make the servo horns slip or
// ur gonna have to do ts again Servo deployment angles configured in config.h:
//   - Drag flaps deployed: 45°
//   - Drag flaps undeployed: 135°
//   - Parachute deployed: 170°
//   - Parachute undeployed: 10°

// Time variables

volatile unsigned long lastT;
volatile unsigned static int deltaT;
unsigned long startTimeStamp = 0;   // ms
unsigned long burnoutTimeStamp = 0; // ms

volatile unsigned long timeElapsed = 0; // time elapsed in flight (ms)
unsigned long timeElapsedSinceBurnout = 0;

volatile int lastMovementT = 0; // ms
volatile float altitude = 0;

// Deployment states
bool dragFlapDeployed = false;
bool parachuteDeployed = false;
bool lastDragFlapDeployed = false;

// flight enable button
bool loopEnabled = false;

bool alreadyPushedData = false;
// States & triggers
// VVV (IMPORTANT) set 0 for testing logging data, 1 for arming the rocket
int flightState = 0;                   // state of the rocket's control
bool logSD = false;
int flightStateAdvancementTrigger = 0; // counts number of times state switching event occurs

int timesTheRocketIsStill = 0;
double altitudeAccel = 0;
// float altitudeV = 0;

double dragCoefficient = 0;
double desiredDragCoef = 0;
double airbrakePower = 0;
double integratedAltitude = 0;
double integratedVelocity = 0;
double trueIntegratedAltitude = 0;
double trueIntegratedVelocity = 0;

double fusedVelo = 0.0;
float accelTrust = 1.0;
// KalmanVelocityEstimator kalman(1, 3);
// RollingAverage airBrakeDragCoef(100);
float CdError;
unsigned long lastAirBrakeDeployTime = 0;
unsigned long lastAirBrakeRetractTime = 0;
float maxAltitude = 0;

float airbrakeAngle = 0;
const float HYSTERESIS = 0.2f;
bool microReady = false; // royan micro adjustmants
int microReadyTime = 0;

bool ejectionChargeUntrust = false;
int ejectionChargeUntrustCount = 0;
int ejectionChargeRecoveryCount = 0;
float ejectionChargePreDipMax = 0;
float lastGoodAltitude = 0;
float lastAltitude = 0;
float secondLastAltitude = 0;

long int TimeStuffT = 0;//used to track timings of stuff
long int SDT = 0;//time it takes for SD to do it's thing logging
long int BaroT = 0;
long int AltProcessT = 0;
long int ADXLT = 0;
long int IMUT = 0;
long int RandomCalculationsT = 0;

#ifdef TEST_MODE
// Provide hardware variants typically defined in other modules
double altitude580 = 0.0;
float newAltitude580 = 0.0f;
double altitudeV580 = 0.0;
float zenith = 0.0f;
float zGlobalAccel = 0.0f;
float xAccel = 0.0f;
float yAccel = 0.0f;
float zAccel = 0.0f;
float GyroY = 0.0f;
float bnoI = 0.0f;
float bnoJ = 0.0f;
float bnoK = 0.0f;
float bnoW = 0.0f;

void appLoop();
void loop() { appLoop(); }

#endif

// everything related to timing
void timeStuff()
{
  // safety so we don't fuck up
  if (micros() <= lastT)
    return;

  // arbitrary delay
  while (micros() - lastT < 5000)
  {
    delayMicroseconds(1);
  }

  unsigned long now = micros();
  deltaT = now - lastT;
  lastT = now;

  unsigned long currentMillis = millis();

  if (flightState == 0)
  { // debug mode
    timeElapsed = millis();
  }
  else if (startTimeStamp == 0)
  { // flight standby
    timeElapsed = 0;
  }
  else
  { // during flight
    timeElapsed = currentMillis - startTimeStamp;
  }

  if (burnoutTimeStamp == 0)
  { // before burnout
    timeElapsedSinceBurnout = 0;
  }
  else
  { // after burnout
    timeElapsedSinceBurnout = currentMillis - burnoutTimeStamp;
  }
}

float predictApogee(float alt, float v, float dragCoef)
{
  return alt + log((dragCoef * v * v / 9.81) + 1.0) / (2.0 * dragCoef);
}

float predictApogeeIdeal(float alt, float v)
{
  return alt + pow(v, 2) / 2. / GRAVITY;
}

float coefOfDrag(float accel, float v)
{
  float coef = abs(accel / v / v);
  return coef;
}

int clampInt(int mini, int maxi, int val)
{ // clamps value between min and max
  return max(mini, min(maxi, val));
}
float clampDouble(float mini, float maxi, float val)
{ // clamps value between min and max
  return max(mini, min(maxi, val));
}

float searchRangeH = 0.1;
float searchRangeL = 0;
float prediction;
float mid;
float inverseApogee(float alt, float desiredApogee, float v)
{ // Working & Tested not any ore

  searchRangeH = 0.1;
  searchRangeL = 0; // SEARCH RANGE IS 0<m<1
  ;
  for (int i = 0; i < 20; i++)
  {
    mid = (searchRangeL + searchRangeH) / 2.0;
    prediction = predictApogee(alt, v, mid);

    if (prediction < desiredApogee)
    { // to the left of desired
      searchRangeH = mid;
    }
    else if (prediction > desiredApogee)
    {
      searchRangeL = mid;
    }
  }
  return mid; // returns drag coef that work
}

// returns percentage from 0 to 1
float dragCoefToAirbrake(double dragCoef)
{
  return ((dragCoef - COEF_DRAG_FLAPUNDEPLOYED) / (COEF_DRAG_FLAPDEPLOYED - COEF_DRAG_FLAPUNDEPLOYED));
}
void openOpenFlap()
{
  dragFlapDeployed = true;
#ifndef TEST_MODE
  // closeSD();
  // openOpenSD();
  openFlaps();
#endif
}
void closeCloseFlap()
{
  dragFlapDeployed = false;
#ifndef TEST_MODE
  // closeSD();
  // openOpenSD();
  closeFlaps();
#endif
}

float timeToGround(float altitude, float altitudeV)
{ // ret
  if (altitude <= 0.0f)
    return 0.0f;
  if (altitudeV >= -0.01f)
    return INFINITY;
  return abs(altitude / -altitudeV);
}

String getDataHeader()
{
  // String datalogHeader =
  return "timeElapsed,"
         "deltaT,"

         "Pressure,"
         "Temp,"

         "Alt,"
         "integratedAlt,"
         "trueIntAlt,"

         "baroAlt580,"

         "fusedVelocity,"
         "integratedVelo,"
         "trueIntegratedVelo,"

         "baroVelo,"

         "altitudeAccel,"
         "bnoGlobalAccel,"
         "yAxisAccel,"

         "Cd,"
         "desired Cd,"
         "Cd error,"
         "perfectAltDeploy,"

         "PredApogee,"
         "airbrake PredApogee,"

         "Chute Deployed,"
         "dragFlaps deployed,"
         "airbrakeAngle,"

         "flight state,"
         "zenith,"
         "roll rate,"
         "i,"
         "j,"
         "k,"
         "w,"
         "xAxisAccel,"
         "yAxisAccel,"

         "\n";
}

#ifdef TEST_MODE
void dataLogging() {}
#else
void dataLogging()
{
  String dataString =
      String(timeElapsed) + ',' +
      String(deltaT) + ',' +

      String(getBaroData_BMP580_pressure(), DATAPRECISION) + ',' +
      String(getBaroData_BMP580_temp(), DATAPRECISION) + ',' +
      String(altitude, DATAPRECISION) + ',' +
      String(integratedAltitude, DATAPRECISION) + ',' +
      String(trueIntegratedAltitude, DATAPRECISION) + ',' +

      String(newAltitude580, DATAPRECISION) + ',' +
      String(fusedVelo, DATAPRECISION) + ',' +
      String(integratedVelocity, DATAPRECISION) + ',' +
      String(trueIntegratedVelocity, DATAPRECISION) + ',' +

      String(altitudeV580, DATAPRECISION) + ',' +
      String(altitudeAccel, DATAPRECISION) + ',' +
      String(zGlobalAccel - GRAVITY, DATAPRECISION) + ',' +
      String(-yAccel - GRAVITY, DATAPRECISION) + ',' +
      String(dragCoefficient, DATAPRECISION) + ',' +
      String(desiredDragCoef, DATAPRECISION) + ',' +
      String(CdError, DATAPRECISION+3) + ',' +
      String((static_cast<float>(DESIRED_FLIGHT_TIME - timeElapsed) / 1000) * PARACHUTE_TERM_VELOCITY, DATAPRECISION) + ',' +

      String(predictApogee(integratedAltitude, fusedVelo, dragCoefficient), DATAPRECISION) + ',' +
      String(predictApogee(integratedAltitude, fusedVelo, COEF_DRAG_FLAPDEPLOYED), DATAPRECISION) + ',' +
      String(parachuteDeployed * 100) + ',' +
      String(dragFlapDeployed * 100) + ',' +
      String(airbrakeAngle, DATAPRECISION) + ',' +
      String(flightState * 100) + ',' +
      String(zenith, DATAPRECISION) + ',' +
      String(GyroY, DATAPRECISION) + ',' +
      String(bnoI, DATAPRECISION) + ',' +
      String(bnoJ, DATAPRECISION) + ',' +
      String(bnoK, DATAPRECISION) + ',' +
      String(bnoW, DATAPRECISION) + ',' +
      String(xAxisAccel) + ',' +
      String(yAxisAccel, DATAPRECISION) + '\n';

  logData(dataString);
}
#endif

double altitudeZero[30] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
double baroAltitudeZero[30] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

double velocityZero[30] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
int zeroIndex = 0;
void warn()
{
  for (int i = 0; i < 3; i++)
  {
    openOpenFlap();
    digitalWrite(2, HIGH);

    delay(500);
    closeCloseFlap();
    digitalWrite(2, LOW);

    delay(500);
  }
  delay(500);
}
void appSetup()
{
#ifdef TEST_MODE
  // in test mode, no hardware initialization
  return;
#else
  Serial.begin(115200);
  Serial.println("Serial Begin");
  pinMode(2, OUTPUT); // blue LED

  //config time over indicator
  byte data[2] = {0x20, 0x02};
  Serial.write(data, sizeof(data));
  Wire.begin(ADXL343_SDA, ADXL343_SCL);
  Wire.setClock(100000);
  bnoSetup();
  Serial.println("BNO Attached");

  adxlSetup();
  Serial.println("ADXL343 Attached");

  baroSetup();
  Serial.println("Barometric Sensor Setup Complete");
  Serial.println("Ground Pressure " + (String)getGroundPressure());
  Serial.println("Ground Temperature " + (String)getGroundTemp());
  servoSetup();

  Serial.end();
  if (!sdSetup())
  {
    warn();
    LedModeError(5, true);

    Serial.println("NO SD CARD VERY BAD");
  }
  else{
    Serial.println("SD INITIALIZED pretty good");

  }
  if (!logSD){
    Serial.begin(115200);

  }
  logData(getDataHeader());


  setupLED();
  LedMode(LED_MODE_PRE_STARTUP);
  LedMode(LED_MODE_ARMING);
#endif

  detachFlaps();
  Serial.println("start \nAwaiting Start Signal...");

  delay(1000);
  if (flightState != 0)
  {
    digitalWrite(2, HIGH);
  }
  delay(1000);
  delay(3000);

  for (int i = 0; i < 10; i++)
  {
    timeStuff();
    IMUdata(deltaT);
    getADXLData();

    delay(100);
  }

  if (flightState == 0){
    attachFlaps();//start flaps
  }

  LedMode(LED_MODE_READY);

  pinMode(FLIGHTENPIN, INPUT_PULLDOWN);

  while(!loopEnabled) {
    if (digitalRead(FLIGHTENPIN) == LOW) {
      loopEnabled = true;
      // Serial.begin(115200);
      lastT = micros();
      clearLED();
      LedMode(LED_MODE_ARMED_READY);

      pinMode(FLIGHTENPIN, OUTPUT);
    }
    delay(100);
  }
}

int paraDeployTime = 0;
void downwardLogic()
{
  // function of altitude and function of altitude for optimal parachute deployment intercept
  // the logic just works out :D
  if (!parachuteDeployed)
  {
    if ((static_cast<float>(DESIRED_FLIGHT_TIME - timeElapsed) / 1000) * PARACHUTE_TERM_VELOCITY >
        altitude)
    {

      deployChute();
      parachuteDeployed = true;
    }
    // safety
    // if the rocket is under 15 meters in the air deploy asap
    if (altitude < 20.0)
    {

      deployChute();
      parachuteDeployed = true;
    }
  }
  else
  { // jiggle physics = on prevents tangling on servo
    if (timeElapsed % 600 > 200)
    {
      deployChute();
    }
    else
    {
      undeployChute();
    }
  }

  if (parachuteDeployed == false)
  {
    paraDeployTime = millis();
  }
  if ((millis() - paraDeployTime) > 2000)
  {
    if (parachuteDeployed)
    { // after parachute deploy micro optimization
      if (!alreadyPushedData)
      {
        alreadyPushedData = true;
      }
      float tGround = timeToGround(altitude, fusedVelo); // sec

      float remainingMs = DESIRED_FLIGHT_TIME - timeElapsed;
      if (remainingMs < 50) // more clamps
        remainingMs = 50;

      float remainingSec = remainingMs / 1000.0f;

      if (altitude <= 5)
      { // if near ground
        closeCloseFlap();
        return;
      }
      if (remainingSec > tGround - HYSTERESIS)
      {
        openOpenFlap();
      }
      else if (remainingSec < tGround + HYSTERESIS)
      {
        closeCloseFlap();
      }
    }
  }
}

double groundAlt = 0;
int lastDeployTime = 0;

float ejectionTimes = 0;
bool ejected = false;

double altitudeCoef = 0;
double xAxisAccel = 0;
double yAxisAccel = 0;
double bernoulliCoefficient = 0;
double bernoulliAltitude = 0;
void appLoop()
{
#ifndef TEST_MODE
  IMUT = micros();
  if (SDT < 3000){//because every ~20 cycles the deltaT jumps to 10 because the sd is flushing or something
    IMUdata(deltaT);//we don't read IMU data to reduce ms from ~14 to 1
  }
  IMUT = IMUT - micros();

  TimeStuffT = micros();    timeStuff();                  TimeStuffT = micros() - TimeStuffT;
  BaroT = micros();         baroDataRead();               BaroT = micros() - BaroT;
  AltProcessT = micros();   altitudeProcessing(deltaT);   AltProcessT = micros() - AltProcessT;
  ADXLT = micros();         getADXLData();                ADXLT = micros() - ADXLT;
  // ledLoop(timeElapsed);
  RandomCalculationsT = micros();
#endif
  // if (accelerometerTime){
  //   altitude = integratedAltitude;
  //   altitudeV = integratedVelocity;
  // }
#ifndef TEST_MODE

  // altitudeV = altitudeV580;
  if ((zenith > 100.0) || (flightState == 0))
  {
#ifdef A1BOARD
    altitudeAccel = axisComponent(getQuat(), xAccel, yAccel, zAccel, "Z+") - GRAVITY; // positive is upward and no gravity factored
#endif
#ifdef A0BOARD
    altitudeAccel = axisComponent(getQuat(), xAccel, yAccel, zAccel, "Z-") - GRAVITY; // positive is upward and no gravity factored
#endif
#ifdef A3BOARD
    altitudeAccel = axisComponent(getQuat(), -xAccel,- yAccel, zAccel, "Z+") - GRAVITY; // positive is upward and no gravity factored
    xAxisAccel = axisComponent(getQuat(), -xAccel,- yAccel, zAccel, "X+"); // positive is upward and no gravity factored
    yAxisAccel = axisComponent(getQuat(), -xAccel,- yAccel, zAccel, "Y+"); // positive is upward and no gravity factored

    #endif
#ifdef A4BOARD
    altitudeAccel = axisComponent(getQuat(), -xAccel,- yAccel, zAccel, "Z+") - GRAVITY; // positive is upward and no gravity factored
    xAxisAccel = axisComponent(getQuat(), -xAccel,- yAccel, zAccel, "X+"); // positive is upward and no gravity factored
    yAxisAccel = axisComponent(getQuat(), -xAccel,- yAccel, zAccel, "Y+"); // positive is upward and no gravity factored

    #endif
  }
  else{
    altitudeAccel = -yAccel - GRAVITY;
  }
#else
  // In TEST_MODE, altitude and altitudeAccel are set by test cases directly.
#endif

  dragCoefficient = abs((altitudeAccel + GRAVITY) / clampDouble(0.01, 100000, pow(fusedVelo, 2)));
  dragCoefficient = clampDouble(0, 1, dragCoefficient); // safety for predict apogee
  CdError = desiredDragCoef - dragCoefficient;
  // altitudeAccel = zGlobalAccel;
  if (timeElapsed < 600){
    bernoulliCoefficient = (newAltitude580-trueIntegratedAltitude)/(pow(trueIntegratedVelocity,2));
  }
  bernoulliAltitude = newAltitude580 -  bernoulliCoefficient*integratedVelocity*integratedVelocity;
  integratedVelocity = (integratedVelocity + (altitudeAccel * deltaT / 1000000.0))*0.9994+altitudeV580*0.0006;
  integratedAltitude = (integratedAltitude + integratedVelocity * deltaT / 1000000.0)*0.98+bernoulliAltitude*0.02;
  trueIntegratedVelocity = trueIntegratedVelocity + (altitudeAccel * deltaT / 1000000.0);
  trueIntegratedAltitude = trueIntegratedAltitude + (trueIntegratedVelocity * deltaT / 1000000.0); 

  altitude = bernoulliAltitude;


  // if ((flightState>1)&&(flightState < 4)){
  //   altitude = integratedAltitude;

  // }
  // else{
  //   altitude = newAltitude580;
  // }
  if (altitude > maxAltitude)
  { // set max altitude
    maxAltitude = altitude;
  }

#ifdef DEBUG
  delay(1000); // you only want a delay because serial printing is gets bogged
               // down
#endif
 
  // main control things
  switch (flightState)
  {
  case 0: // happy data printing mode
    fusedVelo = integratedVelocity;
    desiredDragCoef = inverseApogee(integratedAltitude, DESIRED_APOGEE-((DESIRED_APOGEE-altitude)*altitudeCoef), fusedVelo);
    RandomCalculationsT = micros()-RandomCalculationsT;

    SDT = micros();
    if (logSD){
    dataLogging();
    if (timeElapsed > 30000){
      closeSD();
      openOpenSD();
    }
    }

    SDT = micros()-SDT;




#pragma region
    // // dataLogging();
    // // desiredDragCoef = inverseApogee(integratedAltitude, DESIRED_APOGEE, fusedVelo);
    // if (timeElapsed % 1000 > 500){
    //   openOpenFlap();
    //   // deployChute();
    // }
    // else{
    //   closeCloseFlap();

    //   // undeployChute();
    // }

    Serial.print(">");
    // Serial.print("TimeStuffT: ");
    // Serial.print(TimeStuffT);
    // Serial.print(", ");
    // Serial.print("SDT: ");
    // Serial.print(SDT);
    // Serial.print(", ");
    // Serial.print("BaroT: ");
    // Serial.print(BaroT);
    // Serial.print(", ");
    // Serial.print("AltProcessT: ");
    // Serial.print(AltProcessT);
    // Serial.print(", ");
    // Serial.print("ADXLT: ");
    // Serial.print(ADXLT);
    // Serial.print(", ");
    // Serial.print("IMUT: ");
    // Serial.print(IMUT);
    // Serial.print(", ");
    // Serial.print("RandomCalculationsT: ");
    // Serial.print(RandomCalculationsT);
    // Serial.print(", ");
    // // Serial.print("BMP390: ");
    // // Serial.print(altitude390,DATAPRECISION);
    // // Serial.print(", ");
    // Serial.print("Ground Pres: ");
    // Serial.print(getGroundPressure(),DATAPRECISION);
    // Serial.print(", ");
    Serial.print("deltaT: ");
    Serial.print(deltaT);
    Serial.print(", ");
    Serial.print("BaroAlt: ");
    Serial.print(newAltitude580, DATAPRECISION);
    Serial.print(", ");
    Serial.print("xAccel: ");
    Serial.print(xAxisAccel, DATAPRECISION);
    Serial.print(", ");
    Serial.print("yAccel: ");
    Serial.print(yAxisAccel, DATAPRECISION);
    Serial.print(", ");
    // Serial.print("intAlt: ");
    // Serial.print(integratedAltitude, DATAPRECISION);
    // Serial.print(", ");
    // Serial.print("baroAlt: ");
    // Serial.print(altitudeV580, DATAPRECISION);
    // Serial.print(", ");

    // // Serial.print("BMP390: ");
    // // Serial.print(altitude390,DATAPRECISION);
    // // Serial.print(", ");
    // Serial.print("BMP390Velo: ");
    // Serial.print(altitudeV390,DATAPRECISION);
    // Serial.print(", ");

    // Serial.print("fusedVelo: ");
    // Serial.print(fusedVelo, DATAPRECISION);
    // Serial.print(",");
    // Serial.print("trueIntegratedVelo: ");
    // Serial.print(trueIntegratedVelocity, DATAPRECISION);
    // Serial.print(",");
    // Serial.print("accel: ");
    // Serial.print(altitudeAccel, DATAPRECISION);
    // Serial.print(", ");
    // Serial.print("globalZaccel: ");
    // Serial.print(zGlobalAccel, DATAPRECISION);
    // Serial.print(", ");
    // Serial.print("zenith: ");
    // Serial.print(zenith, DATAPRECISION);
    // Serial.print(", ");
    // Serial.print("dragCoef: ");
    // Serial.print(dragCoefficient, DATAPRECISION);
    // Serial.print(", ");
    // Serial.print("target: ");
    // Serial.print(desiredDragCoef, DATAPRECISION);
    // Serial.print(", ");
    // Serial.print("predApog: ");
    // Serial.print(predictApogee(integratedAltitude,fusedVelo,dragCoefficient), DATAPRECISION);
    // Serial.print(", ");

    // Serial.print("inverseApog: ");
    // Serial.print(inverseApogee(integratedAltitude, DESIRED_APOGEE, integratedVelocity), DATAPRECISION);
    // Serial.print(", ");
    // Serial.print("accelY: ");
    // Serial.print(accelBnoY,DATAPRECISION);
    // Serial.print(", ");
    // Serial.print("GlobalZaccelAdxl: ");
    // Serial.print(axisComponent(getQuat(),xAccel, yAccel, zAccel, "Z+"),DATAPRECISION);
    // Serial.print(", ");
    // Serial.print("veloIntegrated: ");
    // Serial.print(integratedVelocity,DATAPRECISION);
    // Serial.print(", ");
    // Serial.print("altitudeIntegrated: ");
    // Serial.print(integratedAltitude,DATAPRECISION);
    // Serial.print(", ");
    // Serial.print("adxlYaccel: ");
    // Serial.print(altitudeAccel,DATAPRECISION);
    // Serial.print(", ");

    // Serial.print("bno055Yaccel: ");
    // Serial.print(AccY,DATAPRECISION);
    // Serial.print(", ");

    Serial.println();

// Serial.println(deltaT);
#pragma endregion
    break;

  case 1: // pad
#pragma region
    fusedVelo = altitudeV580;
    desiredDragCoef = inverseApogee(integratedAltitude, DESIRED_APOGEE-((DESIRED_APOGEE-altitude)*altitudeCoef), fusedVelo);

    Serial.print("Waiting on Pad");
    Serial.print("Accel: ");
    Serial.print(altitudeAccel);
    Serial.print("  Alt: ");
    Serial.print(altitude);
    Serial.println();
    // detects launch if accel is above 50 m/s^2
    // safety altitude is above 2 meters

    altitudeZero[zeroIndex] = 0;
    baroAltitudeZero[zeroIndex] = newAltitude580;
    velocityZero[zeroIndex] = 0;

    zeroIndex++; // increment to next in rolling array
    zeroIndex %= 30;
    for (int i = 0; i < 30; i++)
    {
      velocityZero[i] += altitudeAccel * deltaT / 1000000.0;
      altitudeZero[i] += velocityZero[i] * deltaT / 1000000.0;
    }

    if (altitudeAccel > 30.0f || altitude > 10.0f) // make sure the altitude accel axis is correct
    {
      flightStateAdvancementTrigger++;
      if (flightStateAdvancementTrigger > 3)
      {
        startTimeStamp = millis();
        LedMode(LED_MODE_IN_AIR);
        integratedVelocity = velocityZero[zeroIndex];
        integratedAltitude = altitudeZero[zeroIndex];
        trueIntegratedVelocity = velocityZero[zeroIndex];
        trueIntegratedAltitude = altitudeZero[zeroIndex];

        groundAlt = baroAltitudeZero[zeroIndex];
        attachFlaps(); // start flaps
        Serial.end();
        flightState++;
      }
    }
    else
    {
      flightStateAdvancementTrigger = 0;
    }
    break;
#pragma endregion
  case 2: // Ascent/Burnout
#pragma region

    SDT = micros();
    dataLogging();
    SDT = micros()-SDT;

#ifdef TEST_MODE
    fusedVelo = altitudeV580;
#else
    fusedVelo = integratedVelocity;
    desiredDragCoef = inverseApogee(integratedAltitude, DESIRED_APOGEE-((DESIRED_APOGEE-altitude)*altitudeCoef), fusedVelo);

#endif
    if (altitudeAccel < 0 || timeElapsed > 2500)
    // warning accel being under 0 does not neccecarily mean that thrust has stopped but it is close enough
    // timeElasped > 2000 is just a safety to make sure it deploys for whatever reason
    // altitudeV < 0.5 is another cooked safety just in the rocket like hella tumbles
    {
      flightStateAdvancementTrigger++;
      if (flightStateAdvancementTrigger > 10)
      {
        burnoutTimeStamp = millis();

        flightState++;
        flightStateAdvancementTrigger = 0;
      }
    }
    else
    {
      flightStateAdvancementTrigger = 0;
    }
    break;
#pragma endregion
  case 3: // coast up
#pragma region

    SDT = micros();
    dataLogging();
    SDT = micros()-SDT;

    // shitty homebrew "filter"
#ifdef TEST_MODE
    fusedVelo = altitudeV580;
#else
    // fusedVelo = altitudeV580 * (1.0 - accelTrust) + integratedVelocity * accelTrust;
    fusedVelo = integratedVelocity;
    desiredDragCoef = inverseApogee(integratedAltitude, DESIRED_APOGEE-((DESIRED_APOGEE-altitude)*altitudeCoef), fusedVelo);

#endif
    // accelTrust -= 0.002; // 1 divided by this amount = roughly how much time till fusedVelo is totally baro
    // accelTrust = clampDouble(0, 1, accelTrust);

    // velocity less than 0.5 means the rocket is going down
    // timeElasped over 10 sec is safety
    if (timeElapsed > 7500)
    {
      flightStateAdvancementTrigger++;
      if (flightStateAdvancementTrigger > 10) // ok lets start the descent
      {
        flightState++;
        flightStateAdvancementTrigger = 0;
        closeSD(); // writes to sd card cuz the hard times are over
        openOpenSD();
        SDT = 0;
        closeCloseFlap(); // stows the airbrakes
        // detach();
      }
      break; // break stops the rest of the code from running
    }
    else
    {
      flightStateAdvancementTrigger = 0;
    }

    // SAFETY CODE
    #pragma region

    //_____________________________________________________________________________________________________________
    // if we are above the desired altitude deploy the flaps (very simple logic)
    if (altitude > DESIRED_APOGEE)
    {
      openOpenFlap();
      break;
    }
    // if we ever need to do this we probably messed up real bad
    // this is just here to save a few points

    // if perfect kinematic predictions say we are below the desired altitude
    // we retract the flaps (somewhat simple logic)
    if (predictApogeeIdeal(altitude, fusedVelo) < DESIRED_APOGEE)
    {
      closeCloseFlap();
      break;
    }
    // if we ever need to rely on this we're probably cooked
    // this is just here to save a few points
    //_____________________________________________________________________________________________________________
    #pragma endregion

    // //stuff right after burnout
    // if (timeElapsedSinceBurnout > 2000)
    // {
    //   openOpenFlap();
    //   break;
    // }
    // else if (timeElapsedSinceBurnout < 2000)
    //   {openOpenFlap();break;}
    
    // oneshot code vv
    // //if the predicted apogee at a certain point with the airbrakes deployed is more than the desired apogee
    // //it would make sense to deploy the airbrakes NOW! so we don't loose more points
    // if (predictApogee(altitude, integratedVelocity, COEF_DRAG_FLAPDEPLOYED) > 228.6)
    // {
    //   openOpenFlap();
    //   lastDeployTime = millis();
    // }
    // else // im sure this will never be the case
    // {
    //   closeCloseFlap();
    //   lastDeployTime = millis();

    // }
    // advanced code vvv
    if (desiredDragCoef > dragCoefficient)
    {
      airbrakeAngle += abs(CdError)*deltaT/1000000.0*2000.0*(airbrakeAngle-DRAG_FLAPS_UNDEPLOYED_ANGLE+60.0);

      // if (abs(CdError) > 0.0050)
      // {
      //   airbrakeAngle += 4;
      // }
      // else if (abs(CdError) > 0.0030)
      // {
      //   airbrakeAngle += 3;
      // }
      // else if (abs(CdError) > 0.0010)
      // {
      //   airbrakeAngle += 2;
      // }
      // else if (abs(CdError) > 0.0005)
      // {
      //   airbrakeAngle += 1;
      // }
      // else if (abs(CdError) > 0.0003)
      // {
      //   airbrakeAngle += 0.5;
      // }
      
      // else
      // {
      //   airbrakeAngle += 0.3;
      // }
    }
    else
    {
      airbrakeAngle -= abs(CdError)*deltaT/1000000.0*2000.0*(DRAG_FLAPS_DEPLOYED_ANGLE-airbrakeAngle+60.0);

      // if (abs(CdError) > 0.0050)
      // {
      //   airbrakeAngle -= 4;
      // }
      // else if (abs(CdError) > 0.0030)
      // {
      //   airbrakeAngle -= 3;
      // }
      // else if (abs(CdError) > 0.0010)
      // {
      //   airbrakeAngle -= 2;
      // }
      // else if (abs(CdError) > 0.0005)
      // {
      //   airbrakeAngle -= 1;
      // }
      // else if (abs(CdError) > 0.0003)
      // {
      //   airbrakeAngle -= 0.5;
      // }
      // else
      // {
      //   airbrakeAngle -= 0.3;
      // }

    }

    airbrakeAngle = clampDouble(DRAG_FLAPS_UNDEPLOYED_ANGLE, DRAG_FLAPS_DEPLOYED_ANGLE, airbrakeAngle);
    moveFlaps((int)round(airbrakeAngle));

    break;
#pragma endregion
  case 4: // we going dow
#pragma region
    fusedVelo = altitudeV580;
    desiredDragCoef = inverseApogee(integratedAltitude, DESIRED_APOGEE-((DESIRED_APOGEE-altitude)*altitudeCoef), fusedVelo);

    closeSD(); // program runs slower now but data is more secure as the sd card closes every cycle
    openOpenSD();
    dataLogging();

    if ((fusedVelo < -15.0) && (!ejected)) // ejection detected
    {
      ejectionTimes = 10; // value needs to be tuned?
      ejected = true;
      lastGoodAltitude = altitude;
    }
    // the thing is ejecting don't trust altitude
    if (ejectionTimes > 0)
    { // ejection correction
      ejectionTimes--;
      altitude = lastGoodAltitude; // hold
    }
    else
    { // timout
      altitude = altitude;
    }

    downwardLogic();

    // checks if the velocity is within 1 & -1 for an extended period of time
    // checks if the position is not near the apogee cause the first statement might trigger at apogee
    // 100 second safety
    if (((-1 < fusedVelo && fusedVelo < 1) &&
         (maxAltitude - 50 > altitude)) ||
        (timeElapsed > 100000))
    {
      timesTheRocketIsStill++;
      if (timesTheRocketIsStill > 30)
      {
        closeSD();      // close the sd card once and for all
        detachFlaps();  // save power unpowering flaps
        detachDeploy(); // save power unpowering flaps
        flightState++;
        ledScoring(maxAltitude, timeElapsed); // start scoring
        if (!alreadyPushedData)
        {
          alreadyPushedData = true;
        }
      }
    }
    else
    {
      timesTheRocketIsStill = 0;
    }

    break;
#pragma endregion
  case 5: // lawn
    delay(10);
    digitalWrite(2, HIGH);
    delay(10);
    digitalWrite(2, LOW);

    break;
  } // end of switch cases
}

/*
If u are reading this it looks like ur the 1 other person who has read the code
persons reading this count: 2
Heres a donut:
                       $@@@$$$######
                     $@@@@@@$$#**!=!!**##
                   #$$@@@@@$$$##!=:::;=!*###
                  *#$$$@@@@@$$$$#*=.~:;;!!*#$$$
                 =*##$$$$@@@@$$$$##*;,-:=!*#$$$@$#
                 =*###$$$$$$$$$$$$$$#*!-~=!*#$$@@@$#
                 ;!!*####$$$$$$$$$$$$$##*!==!*#$$$@$$*
                 ;=!**####$$$$$$$$$$$$$$###**!!*##$$$$#
                  ;=!!***######$$$$$$$$$$$$$############
                  ~;=!!!!**#*######$#$$$$$$$############*
                   ~:;=!!!*!***#*######################**=
                    -~;;=!!!!!********#############****!!=
                      -:;;;==!!!!!!**!****************!!=;
                       .-~::;;====!!!!!!*!!*!**!!!!!!==;;~
                         .--~::;;=;====!!=!==!!=!!===;;:~
                            .,-~~:::;:;;;;=;;==;;;;;::~-
                               ..,--~~~~::::::::::~--.
*/
