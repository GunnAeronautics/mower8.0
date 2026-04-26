#include "headers/utils/servos.h"
#include "headers/config/config.h"
#include "headers/utils/Led.h"

float minServo1 = 0;
float maxServo1 = 100;

#define SERVO1_PIN SERVO_J9 // Duel Deploy // 45 is unarmed, 135 is release parachute
#define SERVO2_PIN SERVO_J7 
#define SERVO3_PIN SERVO_J6 // drag flaps
#define SERVO4_PIN SERVO_J5 // drag flaps
#define SERVO5_PIN SERVO_J4 

// #define MIN_FLAP_STATE 0  // angle for minimum flap
// #define MAX_FLAP_STATE 90 // angle for max flaps
Servo servo1; // create servo
Servo servo2; // create servo
Servo servo3; // create servo
Servo servo4; // create servo

int angleToBlink(double power){// scale of 0 to 90
    return 135 -(int)power;//tony rocket method
  }

void deployChute(){
    servo1.write(PARACHUTE_DEPLOYED_ANGLE);//for drag flaps
  }
void undeployChute(){
    servo1.write(PARACHUTE_UNDEPLOYED_ANGLE);
}
//0 to 1
void moveFlaps(int angle){
    servo2.write(angle);
    servo3.write(angle);
    // servo2.write((int)(power * (DRAG_FLAPS_DEPLOYED_ANGLE - DRAG_FLAPS_UNDEPLOYED_ANGLE) + DRAG_FLAPS_UNDEPLOYED_ANGLE));
    // servo3.write((int)(power * (DRAG_FLAPS_DEPLOYED_ANGLE - DRAG_FLAPS_UNDEPLOYED_ANGLE) + DRAG_FLAPS_UNDEPLOYED_ANGLE));

    // servo4.write(angleToBlink(power));  
}

void attachFlaps(){
  servo2.attach(SERVO2_PIN);
  servo3.attach(SERVO3_PIN);
  // servo1.detach();
}
void detachFlaps(){
  servo2.detach();
  servo3.detach();
  // servo1.detach();
}
void detachDeploy(){
  servo4.detach();
  // servo1.detach();
}

void servoSetup(){
    // servo1.attach(SERVO1_PIN);
    servo1.attach(SERVO1_PIN);
    servo2.attach(SERVO2_PIN);
    servo3.attach(SERVO3_PIN);



    Serial.println("Servo Attached");


    for (int i = 0; i < 180; i++)
    {
      servo2.write(i);
      servo3.write(i);
      // servo4.write(i);
  
      delay(10);
    }
    for (int i = 180; i > 0; i--)
    {
      servo2.write(i);
      servo3.write(i);
      // servo4.write(i);
      delay(10);
    }
    servo1.write(PARACHUTE_UNDEPLOYED_ANGLE);//for drag flaps

    servo2.write(DRAG_FLAPS_UNDEPLOYED_ANGLE);
    servo3.write(DRAG_FLAPS_UNDEPLOYED_ANGLE);
    // servo4.write(DRAG_FLAPS_UNDEPLOYED_ANGLE);

    Serial.println("Servo Test Done");

}
void openFlaps(){
    servo2.write(DRAG_FLAPS_DEPLOYED_ANGLE);
    servo3.write(DRAG_FLAPS_DEPLOYED_ANGLE);
    // servo4.write(DRAG_FLAPS_DEPLOYED_ANGLE);  
}
void closeFlaps(){
    servo2.write(DRAG_FLAPS_UNDEPLOYED_ANGLE);
    servo3.write(DRAG_FLAPS_UNDEPLOYED_ANGLE);
    // servo4.write(DRAG_FLAPS_UNDEPLOYED_ANGLE);  
}
