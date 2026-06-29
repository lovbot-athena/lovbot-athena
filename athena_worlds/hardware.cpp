#include "hardware.h"

// === BUTTON VARIABLES ===
const int BUTTON_SCAN_COUNT = 5;
bool calibrationModeActive = false;
bool previousModeButtonHeld = false;
int buttonToggleState[BUTTON_SCAN_COUNT] = {0};
int buttonWasPressed[BUTTON_SCAN_COUNT] = {0};
bool buttonHoldActive[BUTTON_SCAN_COUNT] = {0};
unsigned long buttonPressedAtMs[BUTTON_SCAN_COUNT] = {0};

// === LED VARIABLES ===
int indicatorMode[INDICATOR_COUNT] = {0, 0, 0, 0, 0};
int indicatorPin[INDICATOR_COUNT] = {33, 34, 35, 36, 37};
long indicatorBlinkAnchorMs[INDICATOR_COUNT] = {0, 0, 0, 0, 0};

// === BUTTON FUNCTIONS ===
void initButtons() {
  for (int i = 0; i < BUTTON_SCAN_COUNT; ++i) {
    pinMode(USER_BUTTON_PINS[i], INPUT_PULLUP);
  }
}

void sampleButtons() {
  for (int i = 0; i < BUTTON_SCAN_COUNT; ++i) {
    int pressedNow = !digitalRead(USER_BUTTON_PINS[i]);
    
    if (pressedNow == 1 && buttonWasPressed[i] == 0) {
      buttonPressedAtMs[i] = millis();
    }
    
    if (pressedNow == 1 && (millis() - buttonPressedAtMs[i]) >= BUTTON_HOLD_MS) {
      buttonHoldActive[i] = true;
    } else if (pressedNow == 0) {
      buttonHoldActive[i] = false;
      buttonPressedAtMs[i] = 0;
    }
    
    if (pressedNow == 1 && buttonWasPressed[i] == 0) {
      buttonToggleState[i] = (buttonToggleState[i] + 1) % BUTTON_MODE_COUNT;
    }
    
    buttonWasPressed[i] = pressedNow;
  }
}

bool buttonHeld(int which) { return buttonHoldActive[which]; }
bool getButtonToggle(int which) { return buttonToggleState[which]; }

void clearButtonToggles() {
  for (int i = 0; i < BUTTON_SCAN_COUNT; ++i) buttonToggleState[i] = 0;
}

void updateButtonLogic() {
  if (buttonHeld(3) && !previousModeButtonHeld) {
    calibrationModeActive = !calibrationModeActive;
  }
  if (calibrationModeActive) {
    Serial.println("calibrating threshold");
    // calibrateBoundaryThreshold(); // This function would be in sensors
  }
  setIndicatorMode(3, calibrationModeActive ? 1 : 0);
  previousModeButtonHeld = buttonHeld(3);
}

// === LED FUNCTIONS ===
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

void setIndicatorMode(int which, int state) {
  if (which >= 0 && which < INDICATOR_COUNT && state >= 0 && state <= 2) {
    indicatorMode[which] = state;
  }
}

void setAllIndicators(int state) {
  for (int i = 0; i < INDICATOR_COUNT; ++i) {
    indicatorMode[i] = state;
    if (state == BLINK) {
      indicatorBlinkAnchorMs[i] = millis();
    }
  }
  serviceIndicators();
}

void clearIndicators() {
  setAllIndicators(OFF);
}

void updateIndicatorLogic() {
  setIndicatorMode(0, boundaryDetected() ? ON : OFF);
  setIndicatorMode(2, getTargetSideMode() == SIDE_A ? ON : OFF);
}

// === SHOOTER FUNCTIONS ===
void initPulseOutput() {
  pinMode(PULSE_OUT_PIN, OUTPUT);
}

void triggerPulseOutput() {
  digitalWrite(PULSE_OUT_PIN, HIGH);
  delay(78); // Kick strength
  digitalWrite(PULSE_OUT_PIN, LOW);
}
