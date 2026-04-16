#ifndef LED_H
#define LED_H

#define NUMPIXELS 2

// LED Modes
#define LED_MODE_PRE_STARTUP 0
#define LED_MODE_ARMING 1
#define LED_MODE_ARMED_READY 2
#define LED_MODE_IN_AIR 3
#define LED_MODE_FATAL_ERROR 4
#define LED_MODE_NONFATAL_ERROR 5

void setupLED();
void LedMode(int mode);
void LedModeError(int code, bool isFatal);
void ledLoop(long timeElapsed);
void ledScoring(float altitude, long timeElapsed);
void clearLED();
void setLEDColor(int r, int g, int b);
void setLED0Color(int r, int g, int b);
void setLED1Color(int r, int g, int b);

#endif // LED_H