//robot.cpp

#include "robot.h"
#include "movement.h"

using namespace Robot;

programStage getNextProgramStage(programStage stage) {
  if (stage != MOVE_TO_LINE_FROM_DROP) {
    return static_cast<programStage>(stage + 1);
  } else {
    return LONG_TRAVERSE_0;
  }
}

//        forward speed, 
//        slow forward speed,
//        turn speed, 
//        turn duration      (last 2 are for line following)  
Vehicle::Vehicle (int fS, int sS, int tS, unsigned long tD) {
  fSpeed = fS;
  motorsActive = false;
  stageShouldAdvance = false;
  currentStage = START;
  crossingDetector = &RisingEdgeDetector();
  lineFollower = &Movement::FollowLine(fS, tS, tD);
  slowForward = &Movement::Straight(sS); // for slow forward movement
  reverse = &Movement::Straight(-fS);
  stopped = &Movement::Stop();
}

void Vehicle::updateCrossingDetector (int lineVal) {
  // add the entry to the rising edge detector 
  crossingDetector->addNew(lineVal == 0b11);
  if (crossingDetector->checkForEdge()) {
   crossingDetector->reset();
   currentCrossingsCount++;
  }
}

// x -> simple, handled by other      v other function?
//    START,                     x  
//    LONG_TRAVERSE_0,           x
//    TURN_TO_BLOCK,             o
//    MOVE_TO_BLOCK,             x
//    SENSE_BLOCK_COLOR,         x    x
//    GRAB_BLOCK,                x    x
//    RAISE_BLOCK                x    x
//    MOVE_TO_LINE_FROM_BLOCK,   o
//    LONG_TRAVERSE_1,           x
//    MOVE_TO_DROP_ZONE,         o
//    LOWER_BLOCK                x    x
//    DROP_BLOCK,                x    x
//    MOVE_TO_LINE_FROM_DROP,    o


// handles the switching of the different movement classes based on the current program stage
Movement::MotorSetting Vehicle::getMotorSetting(int lineVal) {
  switch (currentStage) {
    case START:
      // move forward until the 2nd line is hit
      break;
    case LONG_TRAVERSE_0:
    case LONG_TRAVERSE_1:
      return lineFollower->getMotorSetting(lineVal);
      break;
    case SENSE_BLOCK_COLOR:
    case RAISE_BLOCK:
    case LOWER_BLOCK:
    case DROP_BLOCK:
      return stopped->getMotorSetting();
    case MOVE_TO_BLOCK:
    case GRAB_BLOCK:
      return slowForward->getMotorSetting();
      
    default:
      return stopped->getMotorSetting();
  }
}

// performs a specific function if there is one associated with the current stage
void Vehicle::performFunction() {
  switch (currentStage) {
    case SENSE_BLOCK_COLOR:
      // see what the colour is
      // set correct value for block colour
      // light correct light
      // delay for 5.1 seconds
      stageShouldAdvance = true;
      break;
    case GRAB_BLOCK:
      // have to progressively close the servo
      // can do this inside a loop with delay or something
      // then lift it when stopped
      stageShouldAdvance = true;
      break;
    case RAISE_BLOCK:
      // progressively turn vertical servo in loop
      stageShouldAdvance = true;
      break;
    case LOWER_BLOCK:
      // progressively turn vertical servo in loop
      stageShouldAdvance = true;
    case DROP_BLOCK:
      // progressively opens grabber servo in loop
      stageShouldAdvance = true;
      break;
  }
}

// checks if the programstage should advance given events that have happened
bool Vehicle::checkForAdvance() {
  // if the flag has been set then this is easy
  if (stageShouldAdvance) {return true;}
  
  //otherwise, have to look at params
  switch (currentStage) {
    case LONG_TRAVERSE_0:
    case LONG_TRAVERSE_1:
      if (currentCrossingsCount == 1) {return true;}
      break;
    case START:
      if (currentCrossingsCount == 2) {return true;}
      break;
    case MOVE_TO_BLOCK:
      // check if depth sensor shows block is close enough
      break;
    case TURN_TO_BLOCK:
    case MOVE_TO_LINE_FROM_BLOCK:
    case MOVE_TO_DROP_ZONE:
    case MOVE_TO_LINE_FROM_DROP:
      // these are all more complex needing input from outside
      break;
  }
  return false;
}


// advances the program stage to the next
void Vehicle::advanceStage() {
  // if we've dropped off a block, increment iteration number
  if (currentStage == MOVE_TO_LINE_FROM_DROP) {currentIteration++;}
  currentStage = getNextProgramStage(currentStage);
  // reset all stage-specific flags
  stageShouldAdvance = false;
  currentCrossingsCount = 0;
  
}
