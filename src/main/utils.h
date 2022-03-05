// utils.h
#ifndef UTILS_H
#define UTILS_H
#include <Arduino.h>

enum Mode {USB, BT, BOTH};

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

class Logger {
  public: 
    Logger (unsigned long, Mode);
    unsigned long timeGap;
    unsigned long lastLogTime;
    Mode mode;
    void logln(int);
    void logln(String);
    void log(int);
    void log(String);
};

#endif
