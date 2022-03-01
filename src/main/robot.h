// robot.h
// contains a class to handle the running of the robot through the program

// need a function that takes lineVals over time and increments the crossings count if a new crossing is touched (rising edge detection)

#ifndef ROBOT_H
#define ROBOT_H

#include "utils.h"
#include "movement.h"

namespace Robot {
  enum programStage {
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

  programStage getNextStage(programStage);
  
  class Vehicle {
    public:
      Vehicle(int, int, int, unsigned long); // constructor
      programStage currentStage;
      int currentIteration;
      int blockColor; // TODO: use colour enum
      bool motorsActive;
      int fSpeed;
      bool stageShouldAdvance; // set when something simple happens that indicates next stage should go
      int currentCrossingsCount; //count of the crossings seen in this stage so far

      // a rising edge detector to find when crossings are met
      RisingEdgeDetector* crossingDetector;

      // might make these pointers instead
      Movement::FollowLine* lineFollower;
      Movement::Straight* slowForward;
      Movement::Straight* reverse;
      Movement::Stop* stopped;
  
      void updateCrossingDetector(int);
      void performFunction();
      Movement::MotorSetting getMotorSetting(int);
      bool checkForAdvance(); // run each iteration of main loop to check if the  next stage should be triggered
      void advanceStage();      
      
  };
}

#endif
