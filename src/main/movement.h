// movement.h
// has definitions for all the different movement sets that can run

#ifndef MOVEMENT_H
#define MOVEMENT_H

namespace Movement{
  struct MotorSetting {
    int speeds[2];
    int directions[2];
  };

  class Move {};
  
  class Stop : public Move {
    public:
      MotorSetting getMotorSetting (void);
   };
  
  class Sweep : public Move {};

  class Forward : public Move {
    public:
      Forward (int);
      int fSpeed; // the forward speed
      MotorSetting getMotorSetting (void);
  };
  
  class FollowLine : public Move {
    public:
      FollowLine (int, int, unsigned long);
      int fSpeed;
      bool turning;
      MotorSetting currentTurn;
      int turnAmount;
      unsigned long turnStart;
      unsigned long turnDuration;
      void setParams(int, int, unsigned long);
      MotorSetting getMotorSetting (int);
  };
}



#endif
