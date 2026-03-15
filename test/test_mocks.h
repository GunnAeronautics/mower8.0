#pragma once

#ifdef TEST_MODE

#include "../src/headers/config/config.h"
#include <cmath>
#include <string>
#include <sstream>
#include <algorithm>

#ifndef max
#define max(a,b) std::max((a),(b))
#endif
#ifndef min
#define min(a,b) std::min((a),(b))
#endif

extern float zGlobalAccel;
extern float xAccel;
extern float yAccel;
extern float zAccel;
extern float GyroY;
extern float bnoI;
extern float bnoJ;
extern float bnoK;
extern float bnoW;
extern double altitude580;
extern float newAltitude580;
extern double altitudeV580;
extern float zenith;

constexpr int OUTPUT = 1;
constexpr int HIGH = 1;
constexpr int LOW = 0;

static unsigned long _testMillis = 0;
inline unsigned long millis() { return _testMillis; }
inline unsigned long micros() { return _testMillis * 1000ul; }
inline void delay(unsigned long ms) { _testMillis += ms; }
inline void delayMicroseconds(unsigned int us) { _testMillis += (us + 999) / 1000; }
inline void pinMode(int, int) {}
inline void digitalWrite(int, int) {}

class String {
public:
  std::string data;
  String() = default;
  String(const char *s) : data(s) {}
  String(const std::string &s) : data(s) {}
  String(char c) : data(1, c) {}

  template <typename T>
  String(const T &value) {
    std::ostringstream oss;
    if (std::is_same<T, bool>::value)
      oss << (value ? "1" : "0");
    else
      oss << value;
    data = oss.str();
  }

  String operator+(const String &other) const { return String(data + other.data); }
  String operator+(const char *other) const { return String(data + other); }
  operator std::string() const { return data; }
};

inline String operator+(const char *lhs, const String &rhs) { return String(std::string(lhs) + rhs.data); }

struct SerialMock {
public:
  template <typename T>
  SerialMock &print(const T &) { return *this; }
  template <typename T>
  SerialMock &print(const T &, int) { return *this; }
  template <typename T>
  SerialMock &println(const T &) { return *this; }
  template <typename T>
  SerialMock &println(const T &, int) { return *this; }
  SerialMock &print() { return *this; }
  SerialMock &println() { return *this; }
};

static SerialMock Serial;

inline void closeSD() {}
inline void openOpenSD() {}
inline void openFlaps() {}
inline void closeFlaps() {}
inline void moveFlaps(int) {}
inline void baroDataRead() {}
inline void altitudeProcessing(int) {}
inline void getADXLData() {}
inline void IMUdata(int) {}
inline void adcSetup() {}
inline void bnoSetup() {}
inline void adxlSetup() {}
inline void sdSetup() {}
inline void servoSetup() {}
inline void baroSetup() {}
inline float getBaroData_BMP580_pressure() { return 0.0f; }
inline float getBaroData_BMP580_temp() { return 0.0f; }
inline float getGroundTemp() { return 0.0f; }
inline float getGroundPressure() { return 0.0f; }
inline float axisComponent(float, float, float, float, const char *) { return 0.0f; }
inline void deployChute() {}
inline float getQuat() { return 0.0f; }

#endif // TEST_MODE
