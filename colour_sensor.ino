/*
  Analog input, analog output, serial output

  Reads an analog input pin, maps the result to a range from 0 to 255 and uses
  the result to set the pulse width modulation (PWM) of an output pin.
  Also prints the results to the Serial Monitor.

  The circuit:
  - potentiometer connected to analog pin 0.
    Center pin of the potentiometer goes to the analog pin.
    side pins of the potentiometer go to +5V and ground
  - LED connected from digital pin 9 to ground through 220 ohm resistor

  created 29 Dec. 2008
  modified 9 Apr 2012
  by Tom Igoe

  This example code is in the public domain.

  https://www.arduino.cc/en/Tutorial/BuiltInExamples/AnalogInOutSerial
*/

// These constants won't change. They're used to give names to the pins used:
const int analogInPin1 = A0; //Blue colour LDR voltage (goes down with more light)
const int analogInPin2 = A1;//Red colour LDR voltage
const int analogInPin3 = A2; //OPB704 Voltage (goes down with decreasing distance)
const int analogOutPin = 9; // Analog output pin that the LED is attached to

int sensorValue1 = 0;        // value read from the pot
int sensorValue2 = 0; 
int sensorValue3 = 0;
// value output to the PWM (analog out)

void setup() {
  // initialize serial communications at 9600 bps:
  Serial.begin(9600);
}

void loop() {
  // read the analog in value:
  sensorValue1 = analogRead(analogInPin1);
  // map it to the range of the analog out:
;
  
  sensorValue2 = analogRead(analogInPin2);
  // map it to the range of the analog out:
  sensorValue3 = analogRead(analogInPin3);

  

  // print the results to the Serial Monitor:
  //Serial.print("Blue = ");
  //Serial.print(sensorValue1);
  //Serial.print("\t Red = ");
  //Serial.println(sensorValue2);
  //Serial.print("\t Is it there?");
  //Serial.println(sensorValue3);
  if(sensorValue3 < 300){
    if( sensorValue1 < sensorValue2){
    Serial.println("Blue");
    }
    if( sensorValue1 > sensorValue2){
        Serial.println("red");
    }
  }

  // wait 2 milliseconds before the next loop for the analog-to-digital
  // converter to settle after the last reading:
  delay(2);
}
