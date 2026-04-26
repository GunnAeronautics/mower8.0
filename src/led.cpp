#include "headers/utils/Led.h"
#include "headers/config/config.h"

#include <Adafruit_NeoPixel.h>

int currentMode = LED_MODE_PRE_STARTUP;
bool blink = false;
bool errorBlink = false;
bool isFatalError = false;
bool rgb = false;
int blinkrate = 0;
long lastErrorTime = 0;
long timeElapsed1 = 0;
long lastTime = 0;
long lastTime1 = 0;
int lastErrorCode = 0;
int currentBlinkColor_r = 0, currentBlinkColor_g = 0, currentBlinkColor_b = 0;
int score = 0;
bool scoring = false;
int altitude1 = 0;

int score10s = 0, score1s = 0, altitude10s = 0, altitude1s = 0;

// rgb components

unsigned int red = 255;
unsigned int green = 0;
unsigned int blue = 0;

Adafruit_NeoPixel pixels(NUMPIXELS, Led_Pin, NEO_GRB + NEO_KHZ800);
// Led 1 and 2

void clearLED()
{
    pixels.clear();
    pixels.show();
}

void setLEDColor(int r, int g, int b)
{
    pixels.setPixelColor(0, pixels.Color(r, g, b));
    pixels.setPixelColor(1, pixels.Color(r, g, b));
    pixels.show();
}

void setLED0Color(int r, int g, int b)
{
    pixels.setPixelColor(0, pixels.Color(r, g, b));
    pixels.show();
}

void setLED1Color(int r, int g, int b)
{
    pixels.setPixelColor(1, pixels.Color(r, g, b));
    pixels.show();
}

void setupLED()
{
    pixels.begin();
    pixels.clear();
    setLEDColor(255, 165, 0);
    blink = true;
    pixels.show();
}

void LedMode(int mode)
{
    currentMode = mode;
    errorBlink = false;
    lastTime = 0;

    switch (mode)
    {
    case LED_MODE_PRE_STARTUP:
        blink = true;
        pixels.setBrightness(255);
        setLEDColor(255, 165, 0);
        break;

    case LED_MODE_ARMING:
        blink = true;
        blinkrate = 400;
        currentBlinkColor_r = 0;
        currentBlinkColor_g = 0;
        currentBlinkColor_b = 255;
        pixels.setBrightness(255);
        setLEDColor(0, 0, 255);
        break;

    case LED_MODE_READY:
        blink = true;
        blinkrate = 300;
        currentBlinkColor_r = 0;
        currentBlinkColor_g = 255;
        currentBlinkColor_b = 0;
        pixels.setBrightness(255);
        setLEDColor(0, 255, 0);
        break;

    case LED_MODE_ARMED_READY:
        blink = false;
        rgb = true;
        setLEDColor(50, 50, 50);
        pixels.setBrightness(255);
        break;

    case LED_MODE_IN_AIR:
        rgb = false;
        blink = false;
        pixels.setBrightness(50);
        setLEDColor(0, 255, 0);
        break;

    case LED_MODE_FATAL_ERROR:
        blink = false;
        errorBlink = true;
        isFatalError = true;
        pixels.setBrightness(255);
        setLED0Color(255, 0, 0);
        setLED1Color(0, 0, 0);
        break;

    case LED_MODE_NONFATAL_ERROR:
        blink = false;
        errorBlink = true;
        isFatalError = false;
        pixels.setBrightness(255);
        setLED0Color(0, 0, 255);
        setLED1Color(0, 0, 0);
        break;

    default:
        clearLED();
        blink = false;
        errorBlink = false;
    }
}

void ledScoring(float altitude2, long timeElapsed1)
{
    scoring = true;
    altitude2 *= 3.280839895; // convert m to ft
    if (timeElapsed1 > 39)
    {
        score = (39 - timeElapsed1) * 4;
    }
    else if (timeElapsed1 < 36)
    {
        score = (36 - timeElapsed1) * 4;
    }

    score += abs(altitude2 - (DESIRED_APOGEE * 3.280839895)); // ft
    altitude1 = altitude2;

    score10s = floor(score / 10);
    score1s = score % 10;

    altitude10s = floor(altitude1 / 10);
    altitude1s = altitude1 % 10;
}

void LedModeError(int code, bool isFatal)
{
    lastErrorCode = code;
    if (isFatal)
    {
        LedMode(LED_MODE_FATAL_ERROR);
    }
    else
    {
        LedMode(LED_MODE_NONFATAL_ERROR);
    }
}

void ledLoop(long timeElapsed1)
{  
    Serial.println("kacchan LED LOOP TIME uwu");
    if (errorBlink)
    {
        int digit10s = floor(lastErrorCode / 10);
        int digit1s = lastErrorCode % 10;

        if (isFatalError)
        {
            setLED0Color(255, 0, 0);
        }
        else
        {
            setLED0Color(0, 0, 255);
        }

        // 10
        for (int i = 0; i < digit10s; i++)
        {
            setLED1Color(255, 255, 255);
            delay(1500);
            setLED1Color(0, 0, 0);
            delay(300);
        }

        delay(500);

        // 1
        for (int i = 0; i < digit1s; i++)
        {
            setLED1Color(255, 255, 255);
            delay(750);
            setLED1Color(0, 0, 0);
            delay(300);
        }
        if (timeElapsed1 - lastErrorTime >= errorONTime)
        {
            errorBlink = false;
        }
    }
    else if (rgb && timeElapsed1 - lastTime1 >= blinkratergb)
    {

        if (red == 255 && green < 255 && blue == 0)
        {
            green++;
        }
        else if (green == 255 && red > 0 && blue == 0)
        {
            red--;
        }
        else if (green == 255 && blue < 255 && red == 0)
        {
            blue++;
        }
        else if (blue == 255 && green > 0 && red == 0)
        {
            green--;
        }
        else if (blue == 255 && red < 255 && green == 0)
        {
            red++;
        }
        else if (red == 255 && blue > 0 && green == 0)
        {
            blue--;
        }
        else
        {
            red = 255;
            green = 0;
            blue = 0;
        }

        setLEDColor(red, green, blue);
        lastTime1 = timeElapsed1;
    }
    else if (scoring)
    {
        for (int i = 0; i < altitude10s; i++) // altitude green
        {
            setLEDColor(255, 0, 255);
            delay(1500);
            clearLED();
            delay(300);
        }

        delay(500);

        for (int i = 0; i < altitude1s; i++)
        {
            setLEDColor(255, 0, 255);
            delay(1500);
            clearLED();
            delay(300);
        }

        delay(1000);

        for (int i = 0; i < score10s; i++) // score magenta
        {
            setLEDColor(255, 0, 255);
            delay(1500);
            clearLED();
            delay(300);
        }

        delay(500);

        for (int i = 0; i < score1s; i++)
        {
            setLEDColor(255, 0, 255);
            delay(1500);
            clearLED();
            delay(300);
        }

        delay(1000);
    }
    else if (blink && timeElapsed1 - lastTime >= blinkrate)
    {
        uint32_t currentColor = pixels.getPixelColor(0);
        uint32_t targetColor = pixels.Color(currentBlinkColor_r, currentBlinkColor_g, currentBlinkColor_b);

        if (currentColor == targetColor)
        {
            clearLED();
        }
        else
        {
            setLEDColor(currentBlinkColor_r, currentBlinkColor_g, currentBlinkColor_b);
        }
        lastTime = timeElapsed1;
    }
}
