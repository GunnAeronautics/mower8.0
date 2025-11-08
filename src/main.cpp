// Sup guys this is Tony the main programmer on the ASTRA24-25 tarc team
// make sure to use camelCase for variables and functions
// use ALLCAPS and UNDER_SCORES for definitions
//
#include <Arduino.h>
#include <math.h>

// Configuration and constants
#include "headers/config/config.h"

// Homebrew libraries:
#include "headers/sensors/bmpxxx.h"
#include "headers/sensors/imu.h"
#include "headers/sensors/adxl.h"
#include "headers/utils/sdcard.h"
#include "headers/utils/servos.h"
#include "headers/math/KalmanVelocityEstimator.h"

// Last tuned before 2nd launch 3/22/25 pls don't make the servo horns slip or ur gonna have to do ts again
// Servo deployment angles configured in config.h:
//   - Drag flaps deployed: 45°
//   - Drag flaps undeployed: 135°
//   - Parachute deployed: 170°
//   - Parachute undeployed: 10°

// Time variables
unsigned long lastT;
static int deltaT;
unsigned long startTimeStamp = 0; // ms
unsigned long burnoutTimeStamp = 0; // ms

long timeElapsed = 0;             // time elapsed in flight (ms)
long timeElapsedSinceBurnout = 0;             // time elapsed in flight (ms)

int lastMovementT = 0;            // ms

// Deployment states
bool dragFlapDeployed = false;
bool parachuteDeployed = false;

// States & triggers
// VVV (IMPORTANT) set 0 for testing logging data, 1 for arming the rocket
int flightState = 0;                   // state of the rocket's control
int flightStateAdvancementTrigger = 0; // counts number of times state switching event occurs
int flapDeploymentTrigger = 0;         // counts # of times you should deploy flaps
int numberOfNegatives = 0;
int timesTheRocketIsStill = 0;
float altitudeAccel = 0;
double dragCoefficient = 0;

KalmanVelocityEstimator kalman(1, 3);
RollingAverage noAirBrakeDragCoef(100);
RollingAverage airBrakeDragCoef(100);

// everything related to timing
void timeStuff()
{
  while ((micros()-lastT) < 10000);
  deltaT = micros() - lastT; // deltaT
  lastT = micros();
  // to set a timer for when the rocket launches
  if ((startTimeStamp == 0) && (flightState != 0))
  {
    timeElapsed = 0;
  }
  else
  {
    timeElapsed = millis() - startTimeStamp;
  }

  if (flightState != 3){
    timeElapsedSinceBurnout = 0;
  }
  else{
    timeElapsedSinceBurnout = millis() - burnoutTimeStamp;
  }
}

float predictApogee(float alt, float v, float dragCoef)
{
  return alt + log((dragCoef * v * v / 9.81) + 1) / (2.0 * dragCoef); // copied from mower6.0
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
void writeDataHeader(){
  String datalogHeader = 
  "timeElapsed,"
  "alt,"
  "altV,"
  "altA,"
  "predictApogee nobrakes,"
  "predictApogee brakes,"
  "drag coef,"
  "parachute Deployed,"
  "dragFlaps deployed,"
  "flight state,"
  "time still,"

  "so on"
  "\n";
  logData(datalogHeader);
}

void dataLogging()
{
  BaroData baro = getBaroData();
  String dataString = String(timeElapsed) + ',' +                                                                                   // rocket flight time
                      // String(pressure, DATAPRECISION) + ',' +                                                                  // pressure
                      String(altitude580, DATAPRECISION) + ',' +                                                                  // alt
                      String(altitudeV580, DATAPRECISION) + ',' +                                                                 // velocity - baro derived
                      String(altitudeAccel, DATAPRECISION) + ',' +                                                                         // global axis
                      // String(AccY, DATAPRECISION) + ',' +                                                                           // local axis
                      // String(zenith, DATAPRECISION) + ',' +                                                                         // angle from the vertical
                      String(predictApogee(altitude580, altitudeV580, noAirBrakeDragCoef.getData()), DATAPRECISION) + ',' + // apogee prediction
                      String(predictApogee(altitude580, altitudeV580, airBrakeDragCoef.getData()), DATAPRECISION) + ',' + // apogee prediction

                      String(dragCoefficient) + ',' +
                      (String)parachuteDeployed + ',' +
                      (String)dragFlapDeployed + ',' +
                      (String)flightState + ',' +
                      (String)timesTheRocketIsStill + ',' +

                      "\n";
  logData(dataString);
}
void setup()
{

  Serial.begin(115200);
  Serial.println("Serial Begin");

  baroSetup();
  Serial.println("Barometric Sensor Setup Complete");

  // IMU_BNO055setup();
  // Serial.println("BNO055 Attached");
  adxlSetup();
  Serial.println("ADXL343 Attached");

  Serial.println("Ground Pressure " + (String)getGroundPressure());
  Serial.println("Ground Temperature " + (String)getGroundTemperature());
  lastAltitude = pressToAlt(getGroundPressure());
  lastAltitudeBuiltIn = getAltitude();

  sdSetup();
  writeDataHeader();
  servoSetup();

  lastT = micros();
  Serial.println("start");

  pinMode(2, OUTPUT); // blue LED
  digitalWrite(2, HIGH);

  delay(5000);
}

void downwardLogic()
{
  if (((float(DESIRED_FLIGHT_TIME - timeElapsed) / 1000) * PARACHUTE_TERM_VELOCITY) > getAltitude())
  { // main logic works!!!
    deployChute();
    parachuteDeployed = true;
  }
  if (getAltitude() < 20.)
  { // saftey (falling down and altitude less than 20)
    deployChute();
    parachuteDeployed = true;
  }
}
void upwardLogic()
{
  if (lastMovementT + 400 < millis())
  {
    if ((predictApogeeIdeal(getAltitude(), getAltitudeVelocity()) < DESIRED_APOGEE))
    { // cooked
      moveFlaps(0);
      dragFlapDeployed = false;
    }
    else
    {
      if (getAltitude() > DESIRED_APOGEE)
      { // cooked
        moveFlaps(90);
        dragFlapDeployed = true;
      }
      if ((AccY < 0) && (AccY > -10))
      { // then u are inside of the freefall upward
        if (predictApogee(getAltitude(), getAltitudeVelocity(), coefOfDrag(zAccel, getAltitudeVelocity())) > DESIRED_APOGEE)
        {
          flapDeploymentTrigger++;
        }
        else
        {
          flapDeploymentTrigger = 0;
        }
      }
      if (flapDeploymentTrigger >= 3)
      {
        moveFlaps(90);
        dragFlapDeployed = true;
        lastMovementT = millis();
      }
      else
      {
        moveFlaps(0);
        dragFlapDeployed = false;
      }
    }
  }
}
float accelVelo = 0;
float predAlt = 0;
float error;
int lastAirBrakeDeployTime = 0;
int lastAirBrakeRetractTime = 0;
void loop()
{
  timeStuff();
  baroDataRead();
  altitudeProcessing(deltaT);
  getADXLData();
  altitudeAccel = altitudeA580;//positive is upward and no gravity factored
  dragCoefficient = altitudeAccel/pow(altitudeV580,2);
  
#ifdef DEBUG
  delay(1000); // you only want a delay because serial printing is gets bogged down
#endif

  // Serial.print(", ");

  // Serial.print("Delta T:");
  // Serial.print(deltaT);
  // Serial.println("---------------------");
  // // get data stuff
  // Serial.println("Starting loop");
  // Serial.println("Time Elapsed: " + String(timeElapsed));
  // Serial.println("Delta T: " + String(deltaT));
  // Serial.println("Main BARO ALT: " + String(getAltitude()));

  // Serial.println("BMP580 temp and pressure: " + String(getBaroData_BMP580_temp()) +
  //                ", " + String(getBaroData_BMP580_pressure()));
  // Serial.println("BMP390 temp and pressure: " + String(getBaroData_BMP390_temp()) +
  //                ", " + String(getBaroData_BMP390_pressure()));

  // Serial.println();
  // Serial.println("altitudeProcessing");
  // IMUdata(deltaT);

  // Serial.println(getAltitude());
  // main control things
  switch (flightState)
  {
  case 0: // happy data printing mode
    // dataLogging();
    if (altitude580 > 0.4){
      closeSD();
    }
    #pragma region 
    // dataLogging();
    // accelVelo += (zAccel-9.8) * deltaT / 1000000;
    // error = altitude - predAlt;
    // accelVelo += error / deltaT*1000000 * 0.1;
    // predAlt = altitude + accelVelo *deltaT/1000000;
    // Serial.print(accelVelo);Serial.print(",");
    // Serial.print(altitudeV);Serial.print(",");
    // Serial.print(AccZ-9.8);Serial.print(",");
    // // Serial.print(String(AccX,DATAPRECISION));Serial.print(",");
    // // Serial.print(String(AccY,DATAPRECISION));Serial.print(",");
    // Serial.print(systemCali);Serial.print(",");
    // Serial.print(accelCali);Serial.print(",");
    // Serial.print(gyroCali);Serial.print(",");
    // Serial.print(magCali);Serial.print(",");
    // Serial.print(String(9.8-zAccel,DATAPRECISION));Serial.print(",");
    // Serial.println(deltaT);
    Serial.print(">");
    Serial.print("BMP390: ");
    Serial.print(altitude390,DATAPRECISION);
    Serial.print(", ");

    Serial.print("BMP580: ");
    Serial.print(altitude580,DATAPRECISION);
    Serial.print(", ");

    Serial.print("BMP390Velo: ");
    Serial.print(altitudeV390,DATAPRECISION);
    Serial.print(", ");

    Serial.print("Accel: ");
    Serial.print(-9.8-yAccel,DATAPRECISION);
    Serial.print(", ");

    Serial.print("BMP580Velo: ");
    Serial.print(altitudeV580,DATAPRECISION);
    Serial.println();

    Serial.print("Accel: ");
    Serial.print(altitudeAccel,DATAPRECISION);
    Serial.println();

    Serial.println(deltaT);
    #pragma endregion
    break;
  
  case 1: // on the launch pad waiting to be ignited
    // do main rocket logic hereVVV
    Serial.println("waiting on launch pad");

    // check if rocket is off the ground 
    // VVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVV
    #pragma region 
    if (getAltitude() > 10)
    { // tune these thresholds and statements
      flightStateAdvancementTrigger++;
      if (flightStateAdvancementTrigger > 3)
      { // tune this thresholds
        startTimeStamp = millis(); // start the flight timer here
        flightState++;
      }
    }
    else
    {
      flightStateAdvancementTrigger = 0;
    }
    #pragma endregion
    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

    break;
  case 2:
    dataLogging();

    //checking if rocket's motor burned out
    //============================================================
    #pragma region 
    if ((altitudeAccel < 0)||(altitudeV580 < 0.5)){//rocket is going downwards
      //checking if rocket going down
      flightStateAdvancementTrigger++;
      if (flightStateAdvancementTrigger > 3){
        burnoutTimeStamp = millis();
        flightStateAdvancementTrigger == 0;
        flightState ++;
      }
    }
    else{
      flightStateAdvancementTrigger == 0;
    }
    #pragma endregion
    //============================================================

    
  case 3: // flight
    dataLogging();

    if (altitudeV580 >= 0.5){//rocket is going upward
        //bread and butter
          
        if (timeElapsedSinceBurnout < 1000){
          dragFlapDeployed = false;
        }
        else  if (timeElapsedSinceBurnout < 2000){
          dragFlapDeployed = true;
        }

        else{
        if ((dragFlapDeployed == false)&&(lastAirBrakeRetractTime < (millis()-DRAG_FLAP_ARM_TIME))){
          noAirBrakeDragCoef.newData(dragCoefficient);
        }
        if ((dragFlapDeployed == true)&&(lastAirBrakeDeployTime < (millis()-DRAG_FLAP_ARM_TIME))){
          airBrakeDragCoef.newData(dragCoefficient);
        }

        //safety vvv
        if ((altitude580 > DESIRED_APOGEE)){
          //moveFlaps(100);
          dragFlapDeployed = true;
          lastAirBrakeDeployTime = millis();
        }
        // ^^^
        //cooked saftey
        if (predictApogeeIdeal(altitude580,altitudeV580) < DESIRED_APOGEE){
          dragFlapDeployed = false;
          lastAirBrakeRetractTime = millis();
        }

        //the best part
        else{ 
          if (predictApogee(altitude580,altitudeV580,airBrakeDragCoef.getData()) >= DESIRED_APOGEE){
            //moveFlaps(100);
            dragFlapDeployed = true;
            lastAirBrakeDeployTime = millis();

          }
          else{
            //moveFlaps(0);
            dragFlapDeployed = false;
            lastAirBrakeRetractTime = millis();

          }
        }
        }
      }
        
        
      


    //checking if rocket is past apogee
    //============================================================
    #pragma region 
    if (altitudeV580 < 0.5){//rocket is going downwards
      //checking if rocket going down
      flightStateAdvancementTrigger++;
      if (flightStateAdvancementTrigger > 10){
        flightState ++;
        flightStateAdvancementTrigger == 0;
        closeSD();
        openOpenSD();
      }
    }
    else{
      flightStateAdvancementTrigger == 0;
    }
    #pragma endregion
    //============================================================
    break;
  case 4:
    dataLogging();

    downwardLogic();
    
    //checking if rocket on the ground
    //============================================================
    #pragma region 
    if (((altitudeV580 > -1)&&(altitudeV580 < 1))||(timeElapsed > 100000)){//on the ground or 100 sec has passed
      timesTheRocketIsStill ++;
    }
    else{
      timesTheRocketIsStill = 0;
    }
    if (timesTheRocketIsStill > 30){
      flightState ++;
    }
    #pragma endregion
    //============================================================
  

    break;
  case 5:
    Serial.println("We done");
    break; 
    
  }
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