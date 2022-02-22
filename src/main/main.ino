// Colour detection code
// LED pins have already been initialised to false (initially)

// within pin numbers setup loop
const int csbPin = 9 // based on the actual physical pin number, csb is in fact a light detecting resistor (LDR) that will drop different voltage based on light levels
const int csrPin = 10 // based on the actual physical pin number, same as above

// if a blue object is detected, the blue filter won't pick up anything (blue light filtered out) but the red filter will, so the blue sensor must read true if a blue object is seen, 
// and thus has a red filter in front of it, while the redsensor must read false and thus has a blue filter in front of it

// initialise the colour sensors (under initalisation of optoswitches)
pinMode(csrPin, INPUT) // blue filter in front
pinMode(csbPin, INPUT) // red filter in front

// colour_reader function is ONLY called when there is the white LED shining and the robot has stopped moving!

void colour_reader {
  
  int blue_light = analogRead(csbPin); 
  
  int red_light = analogRead(cbrPin);
  
  if (blue_light > 400 && red_light < 400) { // whatever corresponds to a high or low analogue value, 400 is placeholder value. Might have to change to an OR statement
    digitalWrite(gled.pin, true);
    delay(5200); // 5.2 seconds just to be safe
    digitalWrite(gled.pin, false);
  }
  
  else {
    digitalWrite(rled.pin, true);
    delay(5200);
    digitalWrite(rled.pin, false);
  }

}

// Reference code from the internet:
//const int ledPin = 13;
//const int ldrPin = A0;
//void setup() {
//  Serial.begin(9600);
//  pinMode(ledPin, OUTPUT);
//  pinMode(ldrPin, INPUT);
//}
//
//void loop() {
//  int ldrStatus = analogRead(ldrPin);
//
//  if (ldrStatus <= 400)
//  {
//    digitalWrite(ledPin, HIGH);
//    Serial.print("Its Dark, Turn on the LED:");
//    Serial.println(ldrStatus);
//
//  }
//  else
//  {
//    digitalWrite(ledPin, LOW);
//    Serial.print("Its Bright, Turn off the LED:");
//    Serial.println(ldrStatus);
//  }
