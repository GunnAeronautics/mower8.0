// Sup guys this is Tony the main programmer on the ASTRA24-25 tarc team
// make sure to use camelCase for variables and functions
// use ALLCAPS and UNDER_SCORES for definitions
//
#include <Arduino.h>
#include <math.h>

// Configuration and constants
#include "headers/config/config.h"

// Homebrew libraries:

#include "headers/config/config.h" // Configuration and constants
#include "headers/math/KalmanVelocityEstimator.h"
#include "headers/sensors/adxl.h"
#include "headers/sensors/bmpxxx.h"
#include "headers/sensors/imu.h"
#include "headers/utils/sdcard.h"
#include "headers/utils/servos.h"

// Last tuned before 2nd launch 3/22/25 pls don't make the servo horns slip or
// ur gonna have to do ts again Servo deployment angles configured in config.h:
//   - Drag flaps deployed: 45°
//   - Drag flaps undeployed: 135°
//   - Parachute deployed: 170°
//   - Parachute undeployed: 10°

// Time variables
unsigned long lastT;
volatile unsigned static int deltaT;
unsigned long startTimeStamp = 0;   // ms
unsigned long burnoutTimeStamp = 0; // ms

unsigned long timeElapsed = 0; // time elapsed in flight (ms)
unsigned long timeElapsedSinceBurnout = 0;

unsigned long lastMovementT = 0; // ms

// Deployment states
bool dragFlapDeployed = false;
bool parachuteDeployed = false;

// States & triggers
// VVV (IMPORTANT) set 0 for testing logging data, 1 for arming the rocket
int flightState = 0; // state of the rocket's control
int flightStateAdvancementTrigger =
    0;                         // counts number of times state switching event occurs
int flapDeploymentTrigger = 0; // counts # of times you should deploy flaps
int numberOfNegatives = 0;
int timesTheRocketIsStill = 0;
float altitudeAccel = 0;
double dragCoefficient = 0;

KalmanVelocityEstimator kalman(1, 3);
RollingAverage noAirBrakeDragCoef(100);
RollingAverage airBrakeDragCoef(100);
void timeStuff()
{
  // safety so we don't fuck up
  if (micros() <= lastT)
    return;

  // arbitrary delay
  while (micros() - lastT < 6000)
  {
    delayMicroseconds(10);
  }

  unsigned long now = micros();
  deltaT = now - lastT;
  lastT = now;

  unsigned long currentMillis = millis();

  if (flightState = 0)
  { // debug mode
    timeElapsed = currentMillis;
  }
  else if (startTimeStamp == 0)
  { // flight standby
    timeElapsed = 0;
  }
  else
  { // during flight
    timeElapsed = currentMillis - startTimeStamp;
  }

  if (burnoutTimeStamp = 0)
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
  return alt + log((dragCoef * v * v / 9.81) + 1) / (2.0 * dragCoef);
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

void openOpenFlap()
{
  dragFlapDeployed = true;
  closeSD();
  openOpenSD();
  openFlaps();
}

void closeCloseFlap()
{
  dragFlapDeployed = false;
  closeSD();
  openOpenSD();
  closeFlaps();
}

void writeDataHeader()
{
  String datalogHeader =
      "timeElapsed,"
      "pres390,"
      "pres580,"
      "alt,"
      "alt390,"
      "altV,"
      "altA,"
      "predictApogee nobrakes,"
      "predictApogee brakes,"
      "drag coef,"
      "R Deployed,"
      "dragFlaps deployed,"
      "flight state,"
      "time still,"
      "so on"
      "\n";
  logData(datalogHeader);
}

void dataLogging()
{
  String dataString =
      String(timeElapsed) + ',' +
      String(altitude, DATAPRECISION) + ',' +
      String(altitudeV, DATAPRECISION) + ',' +
      String(altitudeAccel, DATAPRECISION) + ',' +
      String(predictApogee(altitude, altitudeV,
                           noAirBrakeDragCoef.getData()),
             DATAPRECISION) +
      ',' +
      String(predictApogee(altitude, altitudeV,
                           airBrakeDragCoef.getData()),
             DATAPRECISION) +
      ',' +
      String(dragCoefficient, DATAPRECISION) + ',' +
      (String)parachuteDeployed + ',' + (String)dragFlapDeployed + ',' +
      (String)flightState + ',' + (String)timesTheRocketIsStill + ',' + "\n";
  logData(dataString);
}

double accelErrorToZeroVelo = 0;
double accelErrorToZeroVeloADXL = 0;

void downwardLogic()
{
  if ((static_cast<float>(DESIRED_FLIGHT_TIME - timeElapsed) / 1000) *
          PARACHUTE_TERM_VELOCITY >
      altitude)
  {
    deployChute();
    parachuteDeployed = true;
  }

  if (altitude < 20.)
  {
    deployChute();
    parachuteDeployed = true;
  }
}

float accelVelo = 0;
float accelVeloADXL = 0;
float accelPos = 0;
float predAlt = 0;
float error;
unsigned long lastAirBrakeDeployTime = 0;
unsigned long lastAirBrakeRetractTime = 0;
bool core1done = false;
bool core0done = false;

float myGlobalZAccel;
float myGlobalZAccelBNO;

float myZAccel;
float myAccelDerrivedPos;

void TaskCore0(void *pvParameters)
{
  while (true)
  {
    if (!core0done)
    {

      baroDataRead();
      altitudeProcessing(deltaT);

      switch (flightState)
      {
      case 0: // happy data printing mode
// dataLogging();
// if (altitude580 > 0.4){
//   closeSD();
// }
#pragma region
              // dataLogging();
        //
        // dataLogging();
        // closeSD();
        // openOpenSD();

        Serial.print(">");
        Serial.print("DeltaT: ");
        Serial.print(deltaT);
        Serial.print(", ");

        // Serial.print("BMP390: ");
        // Serial.print(altitude390,DATAPRECISION);
        // Serial.print(", ");

        Serial.print("BMP581: ");
        Serial.print(altitude, DATAPRECISION);
        Serial.print(", ");

        Serial.print("BMPVelo: ");
        Serial.print(altitudeV, DATAPRECISION);
        Serial.print(", ");

        // Serial.print("BMP580Velo: ");
        // Serial.print(altitudeV580,DATAPRECISION);
        // Serial.print(",");

        Serial.print("baroAccel: ");
        Serial.print(altitudeA, DATAPRECISION);
        Serial.print(", ");
        // Serial.print("GlobalZaccelBNO: ");
        // Serial.print(myGlobalZAccelBNO, DATAPRECISION);
        // Serial.print(", ");
        Serial.print("GlobalZaccelAdxl: ");
        Serial.print(myGlobalZAccel, DATAPRECISION);
        Serial.print(", ");

        // Serial.print("accelPos: ");
        // Serial.print(myAccelDerrivedPos, DATAPRECISION);
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

      case 1: // Pad
        if (altitude > 10)
        {
          flightStateAdvancementTrigger++;
          if (flightStateAdvancementTrigger > 3)
          {
            startTimeStamp = millis();
            flightState++;
          }
        }
        else
        {
          flightStateAdvancementTrigger = 0;
        }
        break;
      case 2: // Ascent/Burnout
        dataLogging();
        if (myZAccel < 0 || altitudeV < 0.5)
        {
          flightStateAdvancementTrigger++;
          if (flightStateAdvancementTrigger > 3)
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

      case 3: // flight
        dataLogging();

        if (altitudeV >= 0.5)
        {
          if (timeElapsedSinceBurnout < 1000)
          {
            closeCloseFlap();
            lastAirBrakeDeployTime = millis();
          }
          else if (timeElapsedSinceBurnout < 2000)
          {
            openOpenFlap();
          }
          else
          {
            if (altitude > DESIRED_APOGEE)
            {
              openOpenFlap();
            }

            if (predictApogeeIdeal(altitude, altitudeV) <
                DESIRED_APOGEE)
            {
              closeCloseFlap();
            }
            else
            {
              if (predictApogee(altitude, altitudeV,
                                airBrakeDragCoef.getData()) >=
                  DESIRED_APOGEE)
              {
                openOpenFlap();
              }
              else
              {
                closeCloseFlap();
              }
            }
          }

          if (!dragFlapDeployed &&
              lastAirBrakeRetractTime < millis() - DRAG_FLAP_ARM_TIME)
          {
            noAirBrakeDragCoef.newData(dragCoefficient);
          }

          if (dragFlapDeployed &&
              lastAirBrakeDeployTime < millis() - DRAG_FLAP_ARM_TIME)
          {
            airBrakeDragCoef.newData(dragCoefficient);
          }
        }

        if (altitudeV < 0.5)
        {
          flightStateAdvancementTrigger++;
          if (flightStateAdvancementTrigger > 10)
          {
            flightState++;
            flightStateAdvancementTrigger = 0;
            closeSD();
            openOpenSD();
          }
        }
        else
        {
          flightStateAdvancementTrigger = 0;
        }
        break;

      case 4:
        closeSD();
        openOpenSD();
        dataLogging();

        closeCloseFlap();

        downwardLogic();

        if ((altitudeV > -1 && altitudeV < 1) ||
            timeElapsed > 100000)
        {
          timesTheRocketIsStill++;
        }
        else
        {
          timesTheRocketIsStill = 0;
        }

        if (timesTheRocketIsStill > 30)
        {
          flightState++;
        }
        break;

      case 5:
        closeSD();
        Serial.println("We done");
        break;
      }

      core0done = true;
    }
    core0done = true;
  }
  vTaskDelay(1);
}

void TaskCore1(void *pvParameters)
{
  while (true)
  {
    if (!core1done)
    {
      getADXLData();
      IMUdata(deltaT);
      // only integrate if we are actually flying or testing
      if (flightState >= 2 && flightState < 5 || flightState == 0)
      {
        float currentGZ = axisComponent(getQuat(), xAccel, yAccel, zAccel, "Z+");

        // Velocity integration
        accelVelo += (zGlobalAccel - accelErrorToZeroVelo) * deltaT / 1000000.0;
        accelVeloADXL += (currentGZ - accelErrorToZeroVeloADXL) * deltaT / 1000000.0;

        // Position integration
        accelPos += accelVeloADXL * deltaT / 1000000.0;
      }
      else
      {
        // zero if on pad or landed
        accelVelo = 0;
        accelVeloADXL = 0;
        accelPos = 0;
      }

      // clamp
      if (accelVelo > 100 || accelVelo < -100)
        accelVelo = 0;

      core1done = true;
    }

    vTaskDelay(1);
  }
}

void setup()
{
  Serial.begin(115200);
  Serial.println("Serial Begin");

  baroSetup();
  Serial.println("Barometric Sensor Setup Complete");

  Wire.begin(ADXL343_SDA, ADXL343_SCL);

  bnoSetup();
  adxlSetup();
  Serial.println("ADXL343 Attached");

  Serial.println("Ground Pressure " + (String)groundPressure);
  Serial.println("Ground Temperature " + (String)groundTemperature);

  sdSetup();
  writeDataHeader();
  closeSD();
  openOpenSD();

  servoSetup();
  lastT = micros();
  Serial.println("start");
  delay(5000);

  for (int i = 0; i < 10; i++)
  {
    timeStuff();
    IMUdata(deltaT);
    getADXLData();
    delay(100);
  }

  accelErrorToZeroVelo = zGlobalAccel;
  accelErrorToZeroVeloADXL =
      axisComponent(getQuat(), xAccel, yAccel, zAccel, "Z+");

  Serial.print("accel Zero :");
  Serial.println(accelErrorToZeroVelo);
  delay(1000);

  xTaskCreatePinnedToCore(TaskCore0, "Task0", 10000, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(TaskCore1, "Task1", 10000, NULL, 1, NULL, 1);

  lastT = micros();

  pinMode(2, OUTPUT); // blue LED
  digitalWrite(2, HIGH);
}

void loop()
{
  timeStuff();
  // core1 needs = false after core0

  while (!core0done || !core1done)
  {
    delayMicroseconds(10);
  }

  core0done = false;
  core1done = false;

  myGlobalZAccel = axisComponent(getQuat(), xAccel, yAccel, zAccel, "Z+");
  myZAccel = zAccel;
  myAccelDerrivedPos = accelPos;
  myGlobalZAccelBNO = zGlobalAccel;
}

// wow looks like the code is super neat < 200 lines woaw
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