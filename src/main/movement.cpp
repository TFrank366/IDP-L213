#include "movement.h"
#include "utils.h"
#include <Arduino.h>
#include <Adafruit_MotorShield.h>

using namespace Movement;

MotorSetting Movement::getMovement(MoveType moveType, int value) {
  switch (moveType) {
    case STOP:
      return (MotorSetting){.speeds = {0, 0}, .directions = {FORWARD, FORWARD}};
      break;
      
    case STRAIGHT:
      if (value < 0) { // straight reversing
        return (MotorSetting){.speeds = {abs(value), abs(value)}, .directions = {BACKWARD, BACKWARD}};
      } 
      else {
        return (MotorSetting){.speeds = {value, value}, .directions = {FORWARD, FORWARD}};
      }
      break;
      
    case TURN:
      break;
      
    case SPIN:
      // ccw is positive (for now) =============================================================================================================================================
      if (value > 0) {
        return (MotorSetting){.speeds = {value, value}, .directions = {BACKWARD, FORWARD}};
      }
      else {
        return (MotorSetting){.speeds = {abs(value), abs(value)}, .directions = {FORWARD, BACKWARD}};
      }
      break;
    
    case LINE_FOLLOW:
      int speed0 = 210;
      int speedPlus = speed0 + 35;
      int speedMinus = speed0 - 35;
      switch (value) {
        // line only on right sensor
        // have to turn right
        case 0b01: 
          return (MotorSetting){.speeds = {speedPlus, abs(speedMinus)}, .directions = {FORWARD, FORWARD}};
          break;
         // line only on left sensor
         // have to turn left
         case 0b10:
          return (MotorSetting){.speeds = {abs(speedMinus), speedPlus}, .directions = {FORWARD, FORWARD}};
          break;
         default: // ie case 0b00 or 0b11
          // go straight ahead
          return (MotorSetting){.speeds = {speed0, speed0}, .directions = {FORWARD, FORWARD}};
          break;
      }
  }
}
