#ifndef CONFIG_H
#define CONFIG_H


#define FAKE_IMU true
// #define DEBUG

// ============================================================================
// HARDWARE PIN CONFIGURATION
// ============================================================================

// Servo Pins
#define SERVO_K 26 // 1
#define SERVO_L 25 // 2
#define SERVO_M 4 // 3
#define SERVO_N 18 // 4 120.60 41.4 37.9

// BMP581 Barometric Sensor Pins
#define BMP_SCK 13 //clk // scl
#define BMP_MISO 27 //sdo // 
#define BMP_MOSI 14 //sda //
#define BMP_CS 32

// ADXL343 Accelerometer Pins, -- uses i2c now
/*#define ADXL343_SCK 13
#define ADXL343_MISO 12
#define ADXL343_MOSI 11
#define ADXL343_CS 10*/
#define ADXL343_ADR 0x53
#define ADXL343_SCL 22
#define ADXL343_SDA 5

 //BNO055 IMU I2C Configuration



// SD Card SPI Pins
#define SD_SCK 19
#define SD_MISO 21//correct
#define SD_MOSI 3//23
#define SD_CS 23//18

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

// Apogee and flight time targets (ARC competition specifications)
#define DESIRED_APOGEE 228.6        // m (750 ft specified by ARC guidelines)
#define DESIRED_FLIGHT_TIME 37500     // milliseconds (36-39 sec regulation 2026)

// Parachute and drag flap configuration
#define PARACHUTE_TERM_VELOCITY 6.5 // m/s
#define COEF_DRAG_FLAPDEPLOYED 0.002213 // accelY / v / v of the rocket when flaps fully deployed

// Servo deployment angles
#define DRAG_FLAPS_DEPLOYED_ANGLE 180
#define DRAG_FLAPS_UNDEPLOYED_ANGLE 0
//in ms
#define DRAG_FLAP_ARM_TIME 500
#define PARACHUTE_DEPLOYED_ANGLE 180
#define PARACHUTE_UNDEPLOYED_ANGLE 0
// #define 
// ============================================================================
// DATA LOGGING
// ============================================================================

#define MAX_STRING_SIZE 300
#define DATAPRECISION 4  // decimals for SD card printing

#endif // CONFIG_H
