// utils.h
// contains the definitions of utilities, such as stages and communication to Arduino

#ifndef UTILS_H
#define UTILS_H
#include <Arduino.h>

// enum to run a certain code when BLUE or RED is activated
enum Color {RED, BLUE};

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

// an enum for keeping track of program stages
enum programStageName {
  START,                                         //0
  LONG_TRAVERSE_0, // deposit -> collection      //1
  SPIN_180,                                      //2
  TURN_TO_BLOCK,                                 //3
  MOVE_TO_BLOCK, // move slower here             //4
  SENSE_BLOCK_COLOR,                             //5
  PUSH_BLOCK,                                    //6
  GRAB_BLOCK,                                    //7
  RAISE_BLOCK,                                   //8
  MOVE_TO_LINE_FROM_BLOCK,                       //9
  LONG_TRAVERSE_1, // collection -> deposit      //10
  MOVE_TO_DROP_ZONE,                             //11
  LOWER_BLOCK,                                   //12
  DROP_BLOCK,                                    //13
  MOVE_TO_LINE_FROM_DROP,                        //14
  ALIGN_TO_LINE,                                 //15
  STAGE_COUNT                                    //16
};

// for logger
enum Mode {USB, BT, BOTH};

// serial logger class with a configurable delay between sends to prevent overloading the bluetooth connection
class Logger {
  public: 
    Logger (unsigned long gap, Mode m) {
      timeGap = gap;
      mode = m;
    }
    unsigned long timeGap;
    unsigned long lastLogTime;
    Mode mode;

    template<typename T>
    void logln(T value) {
      unsigned long nowTime = millis();
      if (nowTime - lastLogTime >= timeGap) {
        if (mode == BOTH || mode == BT) {SerialNina.println(value);}
        if (mode == BOTH || mode == USB) {Serial.println(value);}
        lastLogTime = nowTime;
      }
    }

    template<typename T>
    void log(T value) {
      unsigned long nowTime = millis();
      if (nowTime - lastLogTime >= timeGap) {
        if (mode == BOTH || mode == BT) {SerialNina.print(value);}
        if (mode == BOTH || mode == USB) {Serial.print(value);}
        lastLogTime = nowTime;
      }
    }
};

// for retrieving data from the line sensors
int getLineVal(Sensor, Sensor);
String getValsString(Sensor, Sensor);

// retreives the colour that the colour sensor sees
Color getColorVal(Sensor, Sensor);

// flashes an led supplied
void flashLed(unsigned long, Led&);

// converts motor speed [0, 255] into speed in m/s
// callibrated at 70 and 150 speed
float speedToReal(int);

#endif
