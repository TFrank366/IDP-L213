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
    START = 0,
    LONG_TRAVERSE_0 = 1, // deposit -> collection
    TURN_TO_BLOCK = 2,
    MOVE_TO_BLOCK = 3, // move slower here
    SENSE_BLOCK_COLOR = 4,
    GRAB_BLOCK = 5,
    RAISE_BLOCK = 6,
    MOVE_TO_LINE_FROM_BLOCK = 7,
    LONG_TRAVERSE_1 = 8, // collection -> deposit
    MOVE_TO_DROP_ZONE = 9,
    LOWER_BLOCK = 10,
    DROP_BLOCK = 11,
    MOVE_TO_LINE_FROM_DROP = 12,
  };

  struct programStage {
     programStageName stageName;
     int next;
  };

  //programStageName getNextStage(programStageName);
  
  class Vehicle {
    public:
      Vehicle(int, int, int, unsigned long); // constructor
      //int currentStageNum;
      //programStageName currentStage;
      int currentIteration;
      int blockColor; // TODO: use colour enum
      bool motorsActive;
      int fSpeed;
      bool stageShouldAdvance; // set when something simple happens that indicates next stage should go
      int currentCrossingsCount; //count of the crossings seen in this stage so far

      // set the program here
      //programStage program[3];
      //int stageNums[3];

      // a rising edge detector to find when crossings are met
      RisingEdgeDetector* crossingDetector;

      // might make these pointers instead
      Movement::FollowLine* lineFollower;
      Movement::Straight* slowForward;
      Movement::Straight* reverse;
      Movement::Stop* stopped;
  
      void updateCrossingDetector(int);
      void performFunction(programStageName, int, Color, Led, Led);
      Movement::MotorSetting getMotorSetting(programStageName, int);
      bool checkForAdvance(programStageName); // run each iteration of main loop to check if the  next stage should be triggered
      void advanceStage();      
      
  };
}

#endif
