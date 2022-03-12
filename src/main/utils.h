// utils.h
// contains the definitions of utilities, such as stages and communication to Arduino

#ifndef UTILS_H
#define UTILS_H
#include <Arduino.h>

enum Mode {USB, BT, BOTH};

// enum to run a certain code when BLUE or RED is activated
enum Color {BLUE, RED};

// Structs for Optosensors and LEDs
struct Sensor {
  int pin;
  int value;
};

struct Led {
  int pin;
  bool state;
  unsigned long interval;
  unsigned long lastChanged;
};

enum programStageName {
  START,                                         //0
  LONG_TRAVERSE_0, // deposit -> collection      //1
  TURN_TO_BLOCK,                                 //2
  MOVE_TO_BLOCK, // move slower here             //3
  SENSE_BLOCK_COLOR,                             //4
  GRAB_BLOCK,                                    //5
  RAISE_BLOCK,                                   //6
  MOVE_TO_LINE_FROM_BLOCK,                       //7
  LONG_TRAVERSE_1, // collection -> deposit      //8
  LOWER_BLOCK,                                   //9
  MOVE_TO_DROP_ZONE,                             //10
  DROP_BLOCK,                                    //11
  MOVE_TO_LINE_FROM_DROP,                        //12
};

class Logger {
  public: 
    Logger (unsigned long, Mode);
    unsigned long timeGap;
    unsigned long lastLogTime;
    Mode mode;
    void logln(int);
    void logln(String);
    void logln(float);
    void logln(double);
    void log(int);
    void log(String);
    void log(float);
    void log(double);
};

int getLineVal(Sensor, Sensor);
String getValsString(Sensor, Sensor);
Color getColorVal(Sensor, Sensor);
void flashLed(unsigned long, Led&);

#endif
