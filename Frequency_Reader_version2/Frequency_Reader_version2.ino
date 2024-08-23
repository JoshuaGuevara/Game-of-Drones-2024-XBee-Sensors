#include <PinChangeInterrupt.h>

const int inputPin = 4;  // Change this to the pin you want to use (4, 5, 6, or 7)
volatile unsigned long pulseCount = 0;
unsigned long lastSeen = 0;

void setup() {
  Serial.begin(57600);
  pinMode(inputPin, INPUT);
  attachPCINT(digitalPinToPCINT(inputPin), countPulse, RISING);
  lastSeen = millis();  // Initialize lastSeen with the current time
}

void loop() {
  //unsigned long currentTime = millis();  // Current time in milliseconds

  if ((millis() - lastSeen) > 100) {  // 100 ms has passed
    float frequency = (pulseCount * 1000.0) / (millis() - lastSeen);  // Calculate frequency in Hz
    if (frequency > 20) {
      Serial.print("Frequency [Hz]: ");
      Serial.println(frequency, 2);  // Print with 2 decimal places
    }

    pulseCount = 0;  // Reset the pulse count
    lastSeen = millis();  // Update lastSeen time
  }
}

void countPulse() {
  pulseCount++;
}

/*
Improvements:
  -More accurate readings; currently reads frequencies (pulse width) in increments of 10
*/