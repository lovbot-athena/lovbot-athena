#include "pulse_output.h"
#include <Arduino.h>

// === Pulse Output State ===
int pulseOutputState = 0;

const int pulseWidthMs = 78;  // Pulse width
const int pulseCooldownMs = 850;     // Minimum delay between pulses

// === Setup ===
void initPulseOutput() {
  pinMode(PULSE_OUT_PIN, OUTPUT);
}

// === Pulse Control ===
void triggerPulseOutput() {
  if (pulseOutputState == OFF) {
    digitalWrite(PULSE_OUT_PIN, HIGH);
    pulseOutputState = ON;
    delay(pulseWidthMs);  // Output pulse window
    digitalWrite(PULSE_OUT_PIN, LOW);
    pulseOutputState = OFF;
    delay(pulseCooldownMs);  // Cooldown window
  }
}


// === Debugging ===
void printPulseOutputState() {
  Serial.print("Time: "); Serial.print(millis());
  Serial.print(" | Status: "); Serial.println(pulseOutputState);
}
