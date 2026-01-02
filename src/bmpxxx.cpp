#include "headers/sensors/bmpxxx.h"
#include <Adafruit_BMP5xx.h>

Adafruit_BMP5xx bmp581;
SPIClass mySPI;

// Rolling averages for sensor data
RollingAverage pressureRoll(8);
RollingAverage temperatureRoll(8);

RollingAverage altitudeVRoll(3);
// Raw sensor data
double groundPressure = 0;
double groundTemperature = 0;



// double lastAltitudeBuiltIn = 0;
// double altitudeBuiltInV = 0;

double temperature = 0, pressure = 0;

double altitude = 0;
double builtInAltitude = 0;

double altitudeV = 0;
double altitudeA = 0;
double lastAltitudeV = 0;
double lastAltitude = 0;

/*
Effective timeline for sensor reading:

~2 ms startup

~38.4 ms oversampling delay for BMP390 at 32x OSR
Limited to ~26 Hz effective read rate for BMP580 at 32x OSR
*/

void baroSetup()
{
  mySPI.begin(BMP_SCK, BMP_MISO, BMP_MOSI);

  if (bmp581.begin(BMP_CS, &mySPI)) {
    Serial.println("No BMP580 found!");
  }

  Serial.println("BMP580 sensor found!");

  bmp581.setPressureOversampling(BMP5XX_OVERSAMPLING_16X);
  bmp581.setIIRFilterCoeff(BMP5XX_IIR_FILTER_COEFF_3);
  bmp581.setOutputDataRate(BMP5XX_ODR_100_2_HZ);
  bmp581.setPowerMode(BMP5XX_POWERMODE_NORMAL);


  // Calibrate rolling averages
  for (int i = 0; i < 100; i++) {
    baroDataRead();
    delay(10);
  }
  
  groundTemperature = temperatureRoll.getData();
  groundPressure = pressureRoll.getData();
}

void baroDataRead()
{
#ifdef SIMULATE
  temperature = 20;
  pressure = 1013.25;
  return;
#endif

  // ---- BMP580 ----
  if (bmp581.performReading()) {
    temperature = bmp581.temperature;
    pressure = bmp581.pressure; // pressure read in hPa
  

  } else {
    Serial.println("BMP580 failed to perform reading!");
  }

  // ---- Combine results ----

  temperatureRoll.newData(temperature);
  pressureRoll.newData(pressure);
  
}


void altitudeProcessing(int deltaT)
{
  altitude = pressToAlt(pressure);
  builtInAltitude = bmp581.readAltitude(groundPressure);

  altitudeVRoll.newData((altitude - lastAltitude) / deltaT * 1000000);
  altitudeV = altitudeVRoll.getData();
  
  altitudeA = ((altitudeV-lastAltitudeV) / deltaT * 1000000);

  lastAltitudeV = altitudeV;
  lastAltitude = altitude;

}


double pressToAlt(double pres)
{
  // Returns altitude (m) from pressure in hecta pascals and temperature in celsius
  // https://www.mide.com/air-pressure-at-altitude-calculator
  // https://en.wikipedia.org/wiki/Barometric_formula
  return (double)(((273.0 + groundTemperature) / (-.0065)) *
                 ((pow((pres / groundPressure),
                       ((8.314 * .0065) / (GRAVITY * .02896)))) -
                  1.0));
}