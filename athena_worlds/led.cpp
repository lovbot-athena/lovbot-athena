#include "led.h"
#include "controller.h"

// Constants
const int INDICATOR_COUNT = 5;
const int INDICATOR_BLINK_MS = 420; // milliseconds

// LED state tracking
int indicatorMode[INDICATOR_COUNT] = {0, 0, 0, 0, 0};
int indicatorPin[INDICATOR_COUNT] = {33, 34, 35, 36, 37};
long indicatorBlinkAnchorMs[INDICATOR_COUNT] = {0, 0, 0, 0, 0};

// Update the physical LED state based on indicatorMode array
void serviceIndicators() {
  long now = millis();

  for (int i = 0; i < INDICATOR_COUNT; ++i) {
    switch (indicatorMode[i]) {
      case OFF:
        digitalWrite(indicatorPin[i], LOW);
        break;

      case ON:
        digitalWrite(indicatorPin[i], HIGH);
        break;

      case BLINK:
        if (now - abs(indicatorBlinkAnchorMs[i]) > INDICATOR_BLINK_MS) {
          if (indicatorBlinkAnchorMs[i] > 0) {
            indicatorBlinkAnchorMs[i] = -now;
            digitalWrite(indicatorPin[i], LOW);
          } else {
            indicatorBlinkAnchorMs[i] = now;
            digitalWrite(indicatorPin[i], HIGH);
          }
        }
        break;
    }
  }
}

// Set individual LED state
void setIndicatorMode(int which, int state) {
  if (which >= 0 && which < INDICATOR_COUNT && state >= 0 && state <= 2) {
    indicatorMode[which] = state;
  }
}

// Set all LEDs to a specified state
void setAllIndicators(int state) {
  for (int i = 0; i < INDICATOR_COUNT; ++i) {
    indicatorMode[i] = state;
    if (state == BLINK) {
      indicatorBlinkAnchorMs[i] = millis();
    }
  }
  serviceIndicators();
}

// Turn off all LEDs
void clearIndicators() {
  setAllIndicators(OFF);
}

// Automatically control LED indicators based on game state
void updateIndicatorLogic() {
  // setIndicatorMode(4, getStartPhaseArmed() ? ON : OFF);          // LED 4 for startPhaseArmed
  setIndicatorMode(0, boundaryDetected() ? ON : OFF);       // LED 0 for boundary detection
  // Additional indicators can be added here
  setIndicatorMode(2, getTargetSideMode()==SIDE_A ? ON : OFF);
}
