#ifndef LED_H
#define LED_H

#define NUMPIXELS 2

#define blinkratergb 2 ///// DELAY BETWEEN EACH CHANG EBLYATTTTTTTTTTTTTTTTT // level converter has rise fall time of 100ns, the neopixel has polling at 125ns, 
#define errorONTime 5000
#define rgbChange 10 /// FUC KING CHANGE BLYATTTTT

// LED Modes
#define LED_MODE_PRE_STARTUP 0
#define LED_MODE_ARMING 1
#define LED_MODE_READY 2 // waiting for button activation
#define LED_MODE_ARMED_READY 3 // on launch pad
#define LED_MODE_IN_AIR 4
#define LED_MODE_FATAL_ERROR 5
#define LED_MODE_NONFATAL_ERROR 6

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