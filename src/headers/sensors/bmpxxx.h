#define BMPXXX_H
#ifdef BMPXXX_H
#include "../config/config.h"
#include "../utils/RollingAverage.h"


// Struct to hold barometric data


// Global data structures
extern float lastAltitude;
extern float lastAltitudeBuiltIn;
extern double altitude390;
extern double altitude580;
extern double altitudeV390;
extern double altitudeV580;
extern double lastAltitudeV580;
extern double altitudeA580;

extern float newAltitude580;
extern float newAltitude390;

// ============================================================================
// INITIALIZATION AND SETUP
// ============================================================================

/**
 * Initialize barometric sensor (auto-detects BMP390 or BMP580)
 * Returns the type of sensor found
 */
void baroSetup();

// ============================================================================
// DATA ACQUISITION
// ============================================================================

/**
 * Read new data from active sensor
 */
void baroDataRead();
void altitudeProcessing(int deltaT);
float getBaroData_BMP580_pressure();
float getBaroData_BMP580_temp();
float getGroundTemp();
float getGroundPressure();

// ============================================================================
// UTILITY FUNCTIONS
// ============================================================================

/**
 * Convert pressure to altitude
 * @param pres Pressure in hecta pascals (hPa)
 * @return Altitude in meters
 */
float pressToAlt(float pres);
// float pressToAlt390(float pres);
// float pressToAlt580(float pres);
float newPressToAlt390(float pres);
float newPressToAlt580(float pres);

#endif // BMPXXX_H
