// robot.h
// contains a class to handle the running of the robot through the program

// ===========================================================================
// need to have a way of creating programs for the robot
// ===========================================================================

#ifndef ROBOT_H
#define ROBOT_H

#include "utils.h"
#include "movement.h"

namespace Robot {  
  enum programStageName {
    START,
    LONG_TRAVERSE_0, // deposit -> collection
    TURN_TO_BLOCK,
    MOVE_TO_BLOCK, // move slower here
    SENSE_BLOCK_COLOR,
    GRAB_BLOCK,
    RAISE_BLOCK,
    MOVE_TO_LINE_FROM_BLOCK,
    LONG_TRAVERSE_1, // collection -> deposit
    MOVE_TO_DROP_ZONE,
    LOWER_BLOCK,
    DROP_BLOCK,
    MOVE_TO_LINE_FROM_DROP,
  };

  struct programStage {
     programStageName stageName;
     int next;
  };

  //programStageName getNextStage(programStageName);
  
  class Vehicle {
    public:
      Vehicle(int, int, int, unsigned long); // constructor
      int currentStageNum;
      int currentIteration;
      int blockColor; // TODO: use colour enum
      bool motorsActive;
      int fSpeed;
      bool stageShouldAdvance; // set when something simple happens that indicates next stage should go
      int currentCrossingsCount; //count of the crossings seen in this stage so far

      // set the program here
      programStage program[3] = {
       (programStage){.stageName=MOVE_TO_BLOCK, .next = 1},
       (programStage){.stageName=SENSE_BLOCK_COLOR, .next = 2},
       (programStage){.stageName=LONG_TRAVERSE_0, .next = 0}
      };

      // a rising edge detector to find when crossings are met
      RisingEdgeDetector* crossingDetector;

      // might make these pointers instead
      Movement::FollowLine* lineFollower;
      Movement::Straight* slowForward;
      Movement::Straight* reverse;
      Movement::Stop* stopped;
  
      void updateCrossingDetector(int);
      void performFunction(int, Color, Led, Led);
      Movement::MotorSetting getMotorSetting(int);
      bool checkForAdvance(); // run each iteration of main loop to check if the  next stage should be triggered
      void advanceStage();      
      
  };
}

#endif
