// utils.h
#ifndef UTILS_H
#define UTILS_H
#include <Arduino.h>

enum Mode {USB, BT, BOTH};

struct Sensor {
  int pin;
  int value;
};

struct Led {
  int pin;
  bool state;
};

enum Color {BLUE, RED};

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

class RisingEdgeDetector {
  public:
    RisingEdgeDetector();
    bool history[10];
    size_t historyLength;
    void addNew(bool); // adds a new entry into the history
    bool checkForEdge(); // look back through history to see if a rising edge is present
    void reset();
};

#endif
