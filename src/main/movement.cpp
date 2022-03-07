#include "movement.h"
#include "utils.h"
#include <Arduino.h>
#include <Adafruit_MotorShield.h>

using namespace Movement;

// takes an input integer that represents state of the sensors as bits (s3 s3 s1) 
// robot is setup m1---m2
//                 \ ^ /
//                  \o/
MotorSetting FollowLine::getMotorSetting(int cols) {
  Serial.println("line following");
  unsigned long nowMillis = millis();
  // create new motorsetting struct
  MotorSetting mSetting;
  // if both are on then stop regardless of whether a turn is in progress
//  if (cols == 0b101 || cols == 0b111) {
//    mSetting = {.speeds = {0, 0}, .directions = {FORWARD, FORWARD}};
//    return mSetting;
//  }
  if (true){//!turning || nowMillis - turnStart > turnDuration) {
    // if there is not a turn taking place or it has elapsed

    //set the turning flag to false
    turning = false;

    int speedPlus = min(fSpeed + turnAmount, 255);
    int speedMinus = fSpeed - turnAmount;
    
    switch (cols) {
      // line only on right sensor
      // have to turn right
      case 0b01: 
        mSetting = {.speeds = {speedPlus, abs(speedMinus)}, .directions = {FORWARD, FORWARD}};
        if (speedMinus < 0) {mSetting.directions[1] = BACKWARD;};
        turning = true;
        turnStart = nowMillis;
        currentTurn = mSetting;
        break;
        
       // line only on left sensor
       // have to turn left
       case 0b10:
        mSetting = {.speeds = {abs(speedMinus), speedPlus}, .directions = {FORWARD, FORWARD}};
        if (speedMinus < 0) {mSetting.directions[0] = BACKWARD;};
        turning = true;
        turnStart = nowMillis;
        currentTurn = mSetting;
        break;
        
       default:
        // go straight ahead
        mSetting = {.speeds = {fSpeed, fSpeed}, .directions = {FORWARD, FORWARD}};
        break;
    }
  } else if (turning) {
    mSetting = currentTurn;
  }

  mSetting = {.speeds = {fSpeed, fSpeed}, .directions = {FORWARD, FORWARD}};
  
  return mSetting;
}

void FollowLine::setParams (int fS, int tAmount, unsigned long tDuration) {
  fSpeed = fS;
  turnAmount = tAmount;
  turnDuration = tDuration;
}

FollowLine::FollowLine (int fS, int tAmount, unsigned long tDuration) {
  fSpeed = fS;
  turnAmount = tAmount;
  turnDuration = tDuration;
  turning = false;
}


MotorSetting Movement::getMovement(MoveType moveType, int value) {
  switch (moveType) {
    case STOP:
      return (MotorSetting){.speeds = {0, 0}, .directions = {FORWARD, FORWARD}};
      break;
    case STRAIGHT:
      if (value < 0) {
        return (MotorSetting){.speeds = {abs(value), abs(value)}, .directions = {BACKWARD, BACKWARD}};
      } else {
        return (MotorSetting){.speeds = {value, value}, .directions = {FORWARD, FORWARD}};
      }
      break;
    case TURN:
    break;
    case SPIN:
      // ccw is positive for now
      if (value > 0) {
        return (MotorSetting){.speeds = {value, value}, .directions = {BACKWARD, FORWARD}};
      } else {
        return (MotorSetting){.speeds = {abs(value), abs(value)}, .directions = {FORWARD, BACKWARD}};
      }
    break;
  }
}
