// movement.h
// has definitions for all the different movement sets that can run

#ifndef MOVEMENT_H
#define MOVEMENT_H

namespace Movement{
  struct MotorSetting {
    int speeds[2]; // 2 speeds, one for each wheel
    int directions[2]; // 2 directions, one for each wheel
  };

// for its various methods of motion
  enum MoveType{
    STOP,
    STRAIGHT,
    TURN,
    SPIN,
    LINE_FOLLOW
 };

  MotorSetting getMovement(MoveType, int);
  
}



#endif
