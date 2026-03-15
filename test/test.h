#ifdef TEST_MODE
#include "test_mocks.h"
#endif

#include "../src/headers/config/config.h"
#include "../src/headers/sensors/bmpxxx.h"

#ifndef TEST_H
#define TEST_H

// core flight state and timing variables
extern int flightState;
extern unsigned int deltaT;
extern unsigned long timeElapsed;
extern unsigned long lastDeployTime;
extern int flightStateAdvancementTrigger;

// altitude/velocity states
extern float altitude;
extern double altitudeV580;
extern double integratedAltitude;
extern double integratedVelocity;
extern double altitudeAccel;

// actuation states
extern bool parachuteDeployed;
extern bool microReady;
extern bool dragFlapDeployed;

// helper routines under test
extern void loop();
extern double predictApogee(double alt, double v, double dragCoef);
extern double predictApogeeIdeal(double alt, double v);
extern void downwardLogic();

#endif
