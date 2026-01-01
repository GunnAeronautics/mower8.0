#ifndef BMPXXX_H
#define BMPXXX_H

#include "../config/config.h"
#include "../utils/RollingAverage.h"


// Struct to hold barometric data


// Global data structures
extern double groundPressure;
extern double groundTemperature;

extern double lastAltitude;
extern double lastAltitudeBuiltIn;
extern double altitude;
extern double altitudeV;
extern double altitudeA;

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

// ============================================================================
// UTILITY FUNCTIONS
// ============================================================================

/**
 * Convert pressure to altitude
 * @param pres Pressure in hecta pascals (hPa)
 * @return Altitude in meters
 */
double pressToAlt(double pres);

#endif // BMPXXX_H
