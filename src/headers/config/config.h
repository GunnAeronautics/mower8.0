#ifndef CONFIG_H
#define CONFIG_H

// #define A0BOARD
// #define A3BOARD
#define A4BOARD

// #define A1BOARD
#define FAKE_IMU false
// #define DEBUG

// ============================================================================
// HARDWARE PIN CONFIGURATION
// ============================================================================

// Servo Pins
#define SERVO_J9 4 // 1 Duel Deploy - left most pin
#define SERVO_J7 18 // 2 
#define SERVO_J6 26 // 3 Airbrakes
#define SERVO_J5 25 // 4 Airbrakes
#define SERVO_J4 33 // 5

// BMP581 Barometric Sensor Pins
#define BMP_SCK 13 //clk // scl
#define BMP_MISO 14 //sdo // 
#define BMP_MOSI 27 //sda //
#define BMP_CS 32

// ADXL343 Accelerometer Pins, -- uses i2c now
/*#define ADXL343_SCK 13
#define ADXL343_MISO 12
#define ADXL343_MOSI 11
#define ADXL343_CS 10*/
#define ADXL343_ADR 0x53 
#define ADXL343_SCL 22
#define ADXL343_SDA 5

 //BNO055 IMU I2C Configuration is the same as ADXL343

// flight enable button
#define FLIGHTENPIN 16

// RGB LED Pin
#define Led_Pin 23


// SD Card SPI Pins
#define SD_SCK 19 // correct
#define SD_MISO 21// correct 
#define SD_MOSI 3//23 rx0
#define SD_CS 1//18 tx0

// ============================================================================
// BNO055 REGISTER CONFIGURATION
// ============================================================================

#define CONFIG_MODE 0x00
#define ACCONLY_MODE 0x01
#define ACC_CONFIG_REGISTER 0x08
#define BNO_ADDRESS 0x28
// ============================================================================
// PHYSICAL CONSTANTS
// ============================================================================
// up is positive y axis

#define GRAVITY 9.81  // m/s^2

// ============================================================================
// FLIGHT PARAMETERS
// ============================================================================

// Apogee and flight time targets (TARC competition specifications)
// #define DESIRED_APOGEE 228.6 //274.3 // 900 ft //228.6      // m (750 ft specified by TARC guidelines) + offset 750 - 723 = 27 ft, binary search 1 2 x 6.5
// #define DESIRED_APOGEE 228.6  //274.3 // 900 ft //228.6      // m (750 ft specified by TARC guidelines) + offset 750 - 723 = 27 ft, binary search 1 2 x 6.5
// #define DESIRED_APOGEE 220.98//725 feet
// #define DESIRED_APOGEE 236.22//775 feet
#define DESIRED_APOGEE 229.198 //274.3 // 900 ft //228.6      // m (750 ft specified by TARC guidelines) + offset 750 - 723 = 27 ft, binary search 1 2 x 6.5

//6.5 765
//2.5 756
#define DESIRED_FLIGHT_TIME 37500    // milliseconds (36-37 sec regulation 2026)
//WARNING DOES NOT ACCOUNT FOR THE BOTTOM SECTION BEING LOWER
//WARNING DOES NOT ACCOUNT FOR THE 
// Parachute and drag flap configuration
// #define PARACHUTE_TERM_VELOCITY 6.5 // m/s
// #define PARACHUTE_TERM_VELOCITY 6.1//spill hole // m/s real // old = 6.65, new = 5.600991463
#define PARACHUTE_TERM_VELOCITY 6.55 // m/s real // old = 6.65, new = 5.600991463

//6.31 works with 3 to 6mph wind 610 G
//5.56 13 mph ~610
#define MASS 0.630//after motor burnout kg
//WARNING THIS IS FROM OPENROCKET 1/17/26

#define COEF_DRAG_FLAPDEPLOYED 0.0040 // accelY / v / v of the rocket when flaps fully deployed
#define COEF_DRAG_FLAPUNDEPLOYED 0.0020 // accelY / v / v of the rocket when flaps fully deployed

// Servo deployment angles
#define DRAG_FLAPS_DEPLOYED_ANGLE 150
#define DRAG_FLAPS_UNDEPLOYED_ANGLE 5
//in ms
#define DRAG_FLAP_MOVE_TIME 500
#define PARACHUTE_DEPLOYED_ANGLE 180
#define PARACHUTE_UNDEPLOYED_ANGLE 0
// #define 
// ============================================================================
// DATA LOGGING
// ============================================================================
#define SLOWBARO false

#define MAX_STRING_SIZE 300
#define DATAPRECISION 4  // decimals for SD card printing

#endif // CONFIG_H
