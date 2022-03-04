// logger.h
#ifndef LOGGER_H
#define LOGGER_H
#include <Arduino.h>

enum Mode {USB, BT, BOTH};

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
