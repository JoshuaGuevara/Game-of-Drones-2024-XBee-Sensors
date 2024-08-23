/*
Why this code doesn't work with arduino xbee code
  -FreqMeasure library conflicts with other libraries needed for xbee & sensors to communicate
  -Pin 8 needs to be in use for this library ()
*/

#include <FreqMeasure.h>

// use pin 8 as input from transimpedance amplifier
// pin 2 as output signal for trouble shooting

void setup() {

pinMode(2, OUTPUT);

Serial.begin(57600);
FreqMeasure.begin();

}

float FreqCountHz;
double sum = 0;
int count = 0; 

int T = 2; //Period in ms (T) = 1/freq

void loop() {

digitalWrite(2, HIGH);
delay(T/2);
digitalWrite(2, LOW);
delay(T/2);

  if(FreqMeasure.available()) {
    sum = sum + FreqMeasure.read();
    count = count + 1;
    if (count > 30) {
      FreqCountHz = FreqMeasure.countToFrequency (sum / count);
      Serial.print(" Frequency[Hz]: ");
      Serial.println(FreqCountHz);
      sum = 0;
      count = 0;
    }
  }
}