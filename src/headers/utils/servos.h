#ifndef SERVOS_H
#define SERVOS_H

#include <Arduino.h>
#include <ESP32Servo.h>
#include "../config/config.h"

void deployChute();
void moveFlaps(int power);
void servoSetup();
void openFlaps();
void closeFlaps();
void undeployChute();
void detachFlaps();
void detachDeploy();
void attachFlaps();
#endif // SERVOS_H
