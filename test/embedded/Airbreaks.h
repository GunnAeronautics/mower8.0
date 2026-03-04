#include "config.h"

#ifndef Airbreaks_h
#define Airbreaks_h

extern int flightState;
extern float DESIRED_APOGEE;
extern float altitudeV;
extern float altitude;

extern int deltaT;
extern int timeElapsed;
extern int lastDeployTime;

extern int flightStateAdvancementTrigger;

extern int integratedAltitude;
extern int integratedVelocity;

extern double altitudeAccel;
extern double dragCoefficient;

// extern Constants
extern float COEF_DRAG_FLAPDEPLOYED;
extern float GRAVITY;
extern float DESIRED_APOGEE;
extern int DESIRED_FLIGHT_TIME;


// Boolean 
extern bool parachuteDeployed; 
extern bool microReady; // delay
extern bool dragFlapDeployed;


// extra variable
extern float remainingSec;

extern void loop();
extern double predictApogee(double alt, double v, double dragCoef);
extern double predictApogeeIdeal(double alt, double v);

#endif
