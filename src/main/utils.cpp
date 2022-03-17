#include "utils.h"
#include <WiFiNINA.h>
#include <SPI.h>
#include <Arduino.h>

int getLineVal(Sensor left, Sensor right) {
  int lineVal = 0;
  lineVal |= analogRead(right.pin) < 20; // change the read values based on line reading ==============================================================
  //delay(10);
  lineVal |= (analogRead(left.pin) < 20) << 1; // change the read values based on line reading ==============================================================
  return lineVal;
}

String getValsString(Sensor left, Sensor right) { // this function is to print out line follower sensor values
  int a  = analogRead(left.pin);
  //delay(10);
  int b  = analogRead(right.pin);
  //delay(10);
  return String(a) + " " + String(b);
}

// gets the colour of what is in front of the colour sensor
// won't be accurate unless distSensor is close enough
Color getColorVal(Sensor rLDR, Sensor bLDR) {
  int bVal = analogRead(bLDR.pin);
  int rVal = analogRead(rLDR.pin);
  if (bVal < rVal) { // Since rVal always seems to be reading more than bVal, change accordingly ==============================================================
    return BLUE;
  } else {
    return RED;
  }
}

void flashLed (unsigned long t, Led& led) {
  if (t - led.lastChanged >= led.interval) {
      // save the last time you blinked the LED
      led.lastChanged = t;
      // set the LED with the ledState of the variable:
      digitalWrite(led.pin, !led.state);
      led.state = !led.state;
    }
}

// linear map based on
// 70  -> 0.02052 m/s
// 150 -> 0.07718 m/s

float speedToReal(int s) {
  if (s == 70) return 0.02052;
  if (s == 150) return 0.078;
  return (s-70)/(150-70) * (0.078-0.02052) + 0.02052;
}
