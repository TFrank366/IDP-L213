#include "movement.h"
#include "utils.h"
#include <Arduino.h>
#include <Adafruit_MotorShield.h>

namespace Movement {
  MotorSetting stop() {
    return (MotorSetting){.speeds = {0, 0}, .directions = {FORWARD, FORWARD}};
  }
  
  MotorSetting straight(int value) {
    if (value < 0) { // straight reversing
      return (MotorSetting){.speeds = {abs(value), abs(value)}, .directions = {BACKWARD, BACKWARD}};
    } 
    else {
      return (MotorSetting){.speeds = {value, value}, .directions = {FORWARD, FORWARD}};
    }
  }
  
  MotorSetting spin(int value) {
    // ccw is positive (for now)
    if (value > 0) {
      return (MotorSetting){.speeds = {value, value}, .directions = {BACKWARD, FORWARD}};
    }
    else {
      return (MotorSetting){.speeds = {abs(value), abs(value)}, .directions = {FORWARD, BACKWARD}};
    }
  }
  
  MotorSetting lineFollow(int value, int lineVal) {
    float turn_sharpness = 0.4; // changes based on the weight
    int speedPlus = value + (int)value*turn_sharpness; // changing sharpness of turn 
    int speedMinus = value - (int)value*turn_sharpness; // changing sharpness of turn
    MotorSetting mSetting;
    switch (lineVal) {
      // line only on right sensor
      // have to turn right
      case 0b01: 
        mSetting = (MotorSetting){.speeds = {speedPlus, abs(speedMinus)}, .directions = {FORWARD, FORWARD}};
        break;
       // line only on left sensor
       // have to turn left
       case 0b10:
        mSetting = (MotorSetting){.speeds = {abs(speedMinus), speedPlus}, .directions = {FORWARD, FORWARD}};
        break;
       default: // ie case 0b00 or 0b11   
        mSetting = (MotorSetting){.speeds = {value, value}, .directions = {FORWARD, FORWARD}};
        break;
    }
    return mSetting;
  }
}
