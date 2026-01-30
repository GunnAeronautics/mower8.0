// Sup guys this is Tony the main programmer on the ASTRA24-25 tarc team
// make sure to use camelCase for variables and functions
// use ALLCAPS and UNDER_SCORES for definitions
//
#include <Arduino.h>
#include <math.h>
#include <SimpleKalmanFilter.h>

// Configuration and constants
#include "headers/config/config.h"

// Homebrew libraries:
// #include "headers/math/KalmanVelocityEstimator.h"
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

int lastMovementT = 0;            // ms
float altitude = 0;

// Deployment states
bool dragFlapDeployed = false;
bool parachuteDeployed = false;
bool lastDragFlapDeployed = false;
// States & triggers
// VVV (IMPORTANT) set 0 for testing logging data, 1 for arming the rocket
int flightState = 0;                   // state of the rocket's control
int flightStateAdvancementTrigger = 0; // counts number of times state switching event occurs

int flapDeploymentTrigger = 0;         // counts # of times you should deploy flaps
int numberOfNegatives = 0;
int timesTheRocketIsStill = 0;
double altitudeAccel = 0;
float altitudeV = 0;

double dragCoefficient = 0;
double integratedAltitude = 0;
double integratedVelocity = 0;
// KalmanVelocityEstimator kalman(1, 3);
// RollingAverage airBrakeDragCoef(100);
float accelVelo = 0;
float accelVeloADXL = 0;
float predAlt = 0;
float error;
unsigned long lastAirBrakeDeployTime = 0;
unsigned long lastAirBrakeRetractTime = 0;
float maxAltitude = 0;

double noAirbrakeDragCoef = 0.0021665;
double AirbrakeDragCoef = 0.0021665*2;

// everything related to timing
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
bool accelerometerTime = false;
void dragCoefProcessing(){
    dragCoefficient = coefOfDrag(altitudeAccel,altitudeV);
    //airbrake deployed for first time
    if (lastDragFlapDeployed == false && dragFlapDeployed == true){
      lastDragFlapDeployed = true;
      lastAirBrakeDeployTime = millis();
      
      accelerometerTime = true;
      integratedAltitude = altitude;
      integratedVelocity = altitudeV;
    }
    //airbrake retracted for first time
    if (lastDragFlapDeployed == true && dragFlapDeployed == false){
      lastDragFlapDeployed = false;
      lastAirBrakeRetractTime = millis();

    }
    if (dragFlapDeployed == false &&
        lastAirBrakeRetractTime < millis() - DRAG_FLAP_MOVE_TIME)//switch back to main altitude
    { //sufficient time has passed so we can start taking measurements for the rolling avwerage
      accelerometerTime = false;
    }

    if (dragFlapDeployed &&
        lastAirBrakeDeployTime < millis() - DRAG_FLAP_MOVE_TIME)
    { //sufficient time has passed so we can start taking measurements for the rolling avwerage
    
      // airBrakeDragCoef.newData(dragCoefficient);
    }
  

}

void openOpenFlap(){
  dragFlapDeployed = true;
  // closeSD();
  // openOpenSD();
  openFlaps();
}

void closeCloseFlap()
{
  dragFlapDeployed = false;
  // closeSD();
  // openOpenSD();
  closeFlaps();
}

void writeDataHeader(){
  String datalogHeader = 
  "timeElapsed,"
  "Pressure,"
  "Temp,"
  "Alt,"
  "integratedAlt,"
  "baroAlt580,"
  "baroAlt390,"
  "mewbaroAlt580,"
  "newbaroAlt390,"

  "velocity,"
  "integratedVelo,"

  "baroVelo,"
  "altitudeAccel,"
  "bnoGlobalAccel,"
  "yAxisAccel,"
  "drag coef,"
  "noAirbrake PredApogee,"
  "airbrake PredApogee,"

  "Chute Deployed,"
  "dragFlaps deployed,"
  "accelerometer time,"

  "flight state,"
  "zenith,"
  "roll rate,"
  "i,"
  "j,"
  "k,"
  "w,"

  "\n";
  logData(datalogHeader);
}

void dataLogging()
{
  String dataString =
      String(timeElapsed) + ',' +  // rocket flight time
      String(getBaroData_BMP580_pressure(), DATAPRECISION) + ',' + // pressure
      String(getBaroData_BMP580_temp(), DATAPRECISION) + ',' + // temp

      String(altitude, DATAPRECISION) + ',' +    // alt

      String(integratedAltitude) + ',' +    // alt
      String(altitude580) + ',' +    // alt
      String(altitude390) + ',' +    // alt
      String(newAltitude580, DATAPRECISION) + ',' +    // alt
      String(newAltitude390, DATAPRECISION) + ',' +    // alt

      String(altitudeV, DATAPRECISION) + ',' +   // velocity - baro derived
      String(integratedVelocity, DATAPRECISION) + ',' +    // alt
      String(altitudeV580, DATAPRECISION) + ',' +   // velocity - baro derived

      String(altitudeAccel, DATAPRECISION) + ',' +  // global axis
      String(zGlobalAccel - GRAVITY, DATAPRECISION) + ',' +  // global axis
      String(-yAccel-GRAVITY, DATAPRECISION) + ',' +  // localAxis

      String(dragCoefficient, DATAPRECISION) + ',' +
      String(predictApogee(altitude, altitudeV,dragCoefficient),DATAPRECISION) + ',' +  // apogee prediction
      String(predictApogee(altitude, altitudeV, 0.0069),DATAPRECISION) + ',' +  // apogee prediction

      (String)(parachuteDeployed*100) + ',' +
      (String)(dragFlapDeployed*100) + ',' +
      (String)(accelerometerTime*100) + ',' +
      (String)(flightState*100) + ',' +

      String(zenith, DATAPRECISION) + ',' +  // localAxis
      String(GyroY, DATAPRECISION) + ',' +  // 

      String(bnoI, DATAPRECISION) + ',' +  // 
      String(bnoJ, DATAPRECISION) + ',' +  // 
      String(bnoK, DATAPRECISION) + ',' +  // 
      String(bnoW, DATAPRECISION) + ',' +  // 

      "\n";
  logData(dataString);
}

double altitudeZero[10] = {0,0,0,0,0,0,0,0,0,0};
double velocityZero[10] = {0,0,0,0,0,0,0,0,0,0};
int zeroIndex = 0;
void setup() {
  Serial.begin(115200);
  Serial.println("Serial Begin");

  baroSetup();
  Serial.println("Barometric Sensor Setup Complete");
  
  Wire.begin(ADXL343_SDA, ADXL343_SCL);

  IMU_BNO055setup();
  // Serial.println("BNO055 Attached");
  adxlSetup();
  Serial.println("ADXL343 Attached");

  Serial.println("Ground Pressure " + (String)getGroundPressure());
  Serial.println("Ground Temperature " + (String)getGroundTemperature());
  lastAltitude = pressToAlt(getGroundPressure());
  lastAltitudeBuiltIn = getAltitude();

  sdSetup();
  writeDataHeader();
  closeSD();
  openOpenSD();
  writeDataHeader();
  closeSD();
  openOpenSD();


  servoSetup();
  lastT = micros();
  Serial.println("start");
  delay(5000);

  for (int i = 0; i < 10; i++){
    timeStuff();
    IMUdata(deltaT);
    getADXLData();

    delay(100);
  }
  delay(1000);
  pinMode(2, OUTPUT);  // blue LED
  digitalWrite(2, HIGH);
  // delay(1000);
  // openOpenFlap();
  // delay(1000);
  // closeCloseFlap();
  // delay(5000);

}

void downwardLogic()
{
  //function of altitude and function of altitude for optimal parachute deployment intercept 
  //the logic just works out :D
  if ((static_cast<float>(DESIRED_FLIGHT_TIME - timeElapsed) / 1000) * PARACHUTE_TERM_VELOCITY >
      altitude)
  {
    deployChute();
    parachuteDeployed = true;
  }

  //safety
  //if the rocket is under 15 meters in the air deploy asap
  if (altitude < 15.0)
  {
    deployChute();
    parachuteDeployed = true;
  }
}


void loop() {
  timeStuff();
  baroDataRead();
  altitudeProcessing(deltaT);
  getADXLData();
  IMUdata(deltaT);
  if (accelerometerTime){
    altitude = integratedAltitude;
    altitudeV = integratedVelocity;
  }
  else{
    altitude = altitude580;
    altitudeV = altitudeV580;
  }
  // integratedVelocity += (zGlobalAccel-9.81)*deltaT/1000000.0;

  altitudeAccel = axisComponent(getQuat(),xAccel, yAccel, zAccel, "Z+") - GRAVITY;  // positive is upward and no gravity factored
  // altitudeAccel = zGlobalAccel;

  integratedVelocity += altitudeAccel*deltaT/1000000.0;
  integratedAltitude += integratedVelocity*deltaT/1000000.0;
  if (altitude > maxAltitude){//set max altitude
      maxAltitude = altitude;
    }

  dragCoefficient = abs(altitudeAccel / pow(altitudeV, 2));

#ifdef DEBUG
  delay(1000);  // you only want a delay because serial printing is gets bogged
                // down
#endif

  // main control things
  switch (flightState)
  {
  case 0: // happy data printing mode
    dataLogging();

    #pragma region 
    // dataLogging();
    //
    // dataLogging();
    closeSD();
    openOpenSD();


    // if ((accelVelo > 100)||(accelVelo < -100)){
    //   accelVelo = 0;
    // }
    Serial.print(">");
    Serial.print("DeltaT: ");
    Serial.print(deltaT);
    Serial.print(", ");

    // Serial.print("BMP390: ");
    // Serial.print(altitude390,DATAPRECISION);
    // Serial.print(", ");

    Serial.print("BMP580: ");
    Serial.print(altitude,DATAPRECISION);
    Serial.print(", ");
    Serial.print("BMP390: ");
    Serial.print(altitude390,DATAPRECISION);
    Serial.print(", ");
    // Serial.print("BMP390Velo: ");
    // Serial.print(altitudeV390,DATAPRECISION);
    // Serial.print(", ");


    Serial.print("BMP580Velo: ");
    Serial.print(altitudeV,DATAPRECISION);
    Serial.print(",");

    Serial.print("baroAccel: ");
    Serial.print(altitudeAccel,DATAPRECISION);
    Serial.print(", ");
    // Serial.print("GlobalZaccel: ");
    // Serial.print(zGlobalAccel,DATAPRECISION);
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

    Serial.print("Waiting on Pad");
    Serial.print("Accel: ");
    Serial.print(altitudeAccel);
    Serial.print("  Alt: ");
    Serial.print(altitude);
    Serial.println();
    //detects launch if accel is above 50 m/s^2
    //safety altitude is above 2 meters

    altitudeZero[zeroIndex] = 0;
    velocityZero[zeroIndex] = 0;

    zeroIndex ++;//increment to next in rolling array
    zeroIndex %= 30;
    for (int i = 0; i < 30; i++){
      velocityZero[i] += altitudeAccel*deltaT/1000000.0;
      altitudeZero[i] += velocityZero[i]*deltaT/1000000.0;
    }

    if (altitudeAccel > 30.0f || altitude > 3.0f)//make sure the altitude accel axis is correct
    {
      flightStateAdvancementTrigger++;
      if (flightStateAdvancementTrigger > 3)
      {
        startTimeStamp = millis();
        
        integratedVelocity = velocityZero[zeroIndex];
        integratedAltitude = altitudeZero[zeroIndex];

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

    dataLogging();
    
    if (altitudeAccel < 0 || timeElapsed > 2000 || altitudeV < 0.5)
    //warning accel being under 0 does not neccecarily mean that thrust has stopped but it is close enough
    //timeElasped > 2000 is just a safety to make sure it deploys for whatever reason
    //altitudeV < 0.5 is another cooked safety just in the rocket like hella tumbles
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
    #pragma endregion
  case 3: // coast up
    #pragma region
    dataLogging();

    //velocity less than 0.5 means the rocket is going down
    //timeElasped over 10 sec is safety
    if (altitudeV <0.5  || timeElapsed > 10000){
      flightStateAdvancementTrigger++;
      if (flightStateAdvancementTrigger > 10)//ok lets start the descent
      {
        flightState++;
        flightStateAdvancementTrigger = 0;
        closeSD();//writes to sd card cuz the hard times are over
        openOpenSD();
        closeCloseFlap(); //stows the airbrakes 

      }
      break;//break stops the rest of the code from running
    }
    else{flightStateAdvancementTrigger = 0;}

    //SAFETY CODE
    //_____________________________________________________________________________________________________________
    //if we are above the desired altitude deploy the flaps (very simple logic)
    if (altitude > DESIRED_APOGEE)
      {openOpenFlap();break;}
    //if we ever need to do this we probably messed up real bad
    //this is just here to save a few points

    //if perfect kinematic predictions say we are below the desired altitude
    //we retract the flaps (somewhat simple logic)
    if (predictApogeeIdeal(altitude, altitudeV) < DESIRED_APOGEE)
      {closeCloseFlap();break;}
    //if we ever need to rely on this we're probably cooked
    //this is just here to save a few points
    //_____________________________________________________________________________________________________________


    // //stuff right after burnout
    // if (timeElapsedSinceBurnout > 1000)
    // {
    //   openOpenFlap();
    //   break;
    // }
    // // else if (timeElapsedSinceBurnout < 2000)
    // //   {openOpenFlap();break;}

    // //if the predicted apogee at a certain point with the airbrakes deployed is more than the desired apogee
    // //it would make sense to deploy the airbrakes NOW! so we don't loose more points
    if (predictApogee(altitude, altitudeV, COEF_DRAG_FLAPDEPLOYED) >= DESIRED_APOGEE)
      {openOpenFlap();}
    else //im sure this will never be the case
      {closeCloseFlap();}
    


    break;
    #pragma endregion
  case 4: // we going down
    #pragma region

    closeSD();//program runs slower now but data is more secure as the sd card closes every cycle
    openOpenSD();
    dataLogging();

    downwardLogic();

    //checks if the velocity is within 1 & -1 for an extended period of time
    //checks if the position is not near the apogee cause the first statement might trigger at apogee 
    //100 second safety
    if (((-1 < altitudeV && altitudeV < 1) &&
         (maxAltitude - 50 > altitude)) ||
        (timeElapsed > 100000))
    {
      timesTheRocketIsStill++;
      if (timesTheRocketIsStill > 30)
        {
          closeSD();// close the sd card once and for all 
          flightState++;
        }
    }
    else
    {
      timesTheRocketIsStill = 0;
    }

    
    break;
    #pragma endregion
  case 5: // lawn
    Serial.println("We Done ");
    Serial.print("maxAlt: ");
    Serial.print(maxAltitude);
    Serial.println();
    break;
  }//end of switch cases
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