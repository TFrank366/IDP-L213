#include "movement.h"
#include "logger.h"
#include <Arduino.h>
#include <Adafruit_MotorShield.h>

using namespace Movement;

// takes an input integer that represents state of the sensors as bits (s3 s3 s1) 
// robot is setup m1---m2
//                 \ ^ /
//                  \o/
MotorSetting FollowLine::getMotorSetting(int cols) {
  unsigned long nowMillis = millis();
  // create new motorsetting struct
  MotorSetting mSetting;
  if (!turning || nowMillis - turnStart > turnDuration) {
    // if there is not a turn taking place or it has elapsed

    //set the turning flag to false
    turning = false;

    int speedPlus = min(fSpeed + turnAmount/2, 255);
    int speedMinus = fSpeed - turnAmount/2;
    
    switch (cols) {
      // line only on right sensor
      // have to turn right
      case 1:
      case 3:  
        mSetting = {.speeds = {speedPlus, abs(speedMinus)}, .directions = {FORWARD, FORWARD}};
        if (speedMinus < 0) {mSetting.directions[1] = BACKWARD;};
        turning = true;
        turnStart = nowMillis;
        currentTurn = mSetting;
        break;
        
       // line only on left sensor
       // have to turn left
       case 4:
       case 6:
        mSetting = {.speeds = {abs(speedMinus), speedPlus}, .directions = {FORWARD, FORWARD}};
        if (speedMinus < 0) {mSetting.directions[0] = BACKWARD;};
        turning = true;
        turnStart = nowMillis;
        currentTurn = mSetting;
        break;
        
       case 5:
       case 7:
        mSetting = {.speeds = {0, 0}, .directions = {FORWARD, FORWARD}};
        break;
        
       default:
        // go straight ahead
        mSetting = {.speeds = {fSpeed, fSpeed}, .directions = {FORWARD, FORWARD}};
        break;
    }
  } else if (turning) {
    mSetting = currentTurn;
  }
  return mSetting;
}

FollowLine::FollowLine (int fS, int tAmount, unsigned long tDuration) {
  fSpeed = fS;
  turnAmount = tAmount;
  turnDuration = tDuration;
  turning = false;
}

MotorSetting Stop::getMotorSetting(void) {
  return (MotorSetting){.speeds = {0, 0}, .directions = {FORWARD, FORWARD}};
}

MotorSetting Forward::getMotorSetting(void) {
  return (MotorSetting){.speeds = {fSpeed, fSpeed}, .directions = {FORWARD, FORWARD}};
}

Forward::Forward (int s) {
  fSpeed = min(max(s, 0), 255); 
}
