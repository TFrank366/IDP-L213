// movement.h
// has definitions for all the different movement sets that can run

#ifndef MOVEMENT_H
#define MOVEMENT_H

namespace Movement{
  struct MotorSetting {
    int speeds[2];
    int directions[2];
  };

  float spinRateTo(int);

  class Move {};
  
  class Stop : public Move {
    public:
      MotorSetting getMotorSetting (void);
   };
  
  class Sweep : public Move {};

  class Straight : public Move {
    public:
      Straight (int);
      Straight ();
      int lSpeed; // the linear speed
      MotorSetting getMotorSetting (void);
  };

  class Spin : public Move {
    public:
      Spin (int);
      FollowLine ();
      int spinRate; // > 0 => CW and vice versa 
      MotorSetting getMotorSetting (void);
  };
  
  class FollowLine : public Move {
    public:
      FollowLine (int, int, unsigned long);
      FollowLine ();
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
