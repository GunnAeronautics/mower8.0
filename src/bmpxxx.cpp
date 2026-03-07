#include "headers/sensors/bmpxxx.h"
#include <Adafruit_BMP5xx.h>
#include <stdexcept>


Adafruit_BMP5xx bmp581;
SPIClass mySPI;

// Rolling averages for sensor data
RollingAverage pressureRoll(8);
RollingAverage temperatureRoll(8);

RollingAverage altitudeVRoll(8);
// Raw sensor data
float groundPressure = 0;
float groundTemperature = 0;



// double lastAltitudeBuiltIn = 0;
// double altitudeBuiltInV = 0;

float temp_390 = 0, pressure_390 = 0;
float temp_580 = 0, pressure_580 = 0;
double altitude390 = 0;
double altitude580 = 0;
float newAltitude580 = 0;
float newAltitude390 = 0;

double altitudeV390;
double altitudeV580;
double altitudeA580;
float lastAltitude580 = 0;
double lastAltitudeV580;
double groundPressure390;
double groundPressure580;

bool bmp390_success = false;
bool bmp580_success = false;

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
    temp_580 = bmp581.temperature;
    pressure_580 = bmp581.pressure; // pressure read in hPa
    temperatureRoll.newData(temp_580);
    pressureRoll.newData(pressure_580);


  } else {
    Serial.println("BMP580 failed to perform reading!");
  }

  // ---- Combine results ----

  
}


//altitude580 is altitude
//altitudeV580 is velocity
//altitudeA580 is acceleration
void altitudeProcessing(int deltaT)
{

  // if ( pressure_580 == NAN ) {
  //       throw std::invalid_argument("Value cannot be Nan");
  // }

  // altitude390 = pressToAlt390(pressure_390);
  altitude580 = pressToAlt(pressure_580);
  // newAltitude390 = newPressToAlt390(pressure_390);
  newAltitude580 = newPressToAlt580(pressure_580);
  


  // altitudeVRoll390.newData((altitude390 - lastAltitude390) / deltaT * 1000000);
  altitudeVRoll.newData((newAltitude580 - lastAltitude580) / deltaT * 1000000);

  // altitudeV390 = altitudeVRoll390.getData();
  altitudeV580 = altitudeVRoll.getData();
  
  altitudeA580 = ((newAltitude580-lastAltitudeV580) / deltaT * 1000000);

  lastAltitudeV580 = altitudeV580;
  lastAltitude580 = newAltitude580;

}


float pressToAlt(float pres)
{
  // Returns altitude (m) from pressure in hecta pascals and temperature in celsius
  // https://www.mide.com/air-pressure-at-altitude-calculator
  // https://en.wikipedia.org/wiki/Barometric_formula
  return (float)(((273.0 + groundTemperature) / (-.0065)) *
                 ((pow((pres / groundPressure),
                       ((8.314 * .0065) / (GRAVITY * .02896)))) -
                  1.0));
}
float newPressToAlt390(float pres)
{
  return (float)(44330.0 * (1.0 - pow(pres / groundPressure390, 0.1903)));
}
float newPressToAlt580(float pres)
{
  return 44330.0 * (1.0 - pow((pres / groundPressure), 0.1903));
}

float getGroundPressure()
{
  return groundPressure;
}

float getGroundTemp()
{
  return groundTemperature;
}

float getBaroData_BMP390_temp()
{
  return temp_390;
}

float getBaroData_BMP390_pressure()
{
  return pressure_390;
}

float getBaroData_BMP580_pressure()
{
  return pressure_580;
}

float getBaroData_BMP580_temp()
{
  return temp_580;
}

