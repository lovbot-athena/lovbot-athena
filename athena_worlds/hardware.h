#ifndef HARDWARE_H
#define HARDWARE_H

#include <Arduino.h>
#include "core.h"

// === BUTTON CONSTANTS ===
#define USER_BUTTON_COUNT 5
#define BUTTON_MODE_COUNT 2
#define BUTTON_HOLD_MS 140

// === LED CONSTANTS ===
#define INDICATOR_COUNT 5
#define INDICATOR_BLINK_MS 420

// === SHOOTER CONSTANTS ===
#define PULSE_OUT_PIN A12

// === BUTTON FUNCTIONS ===
void initButtons();
void sampleButtons();
bool buttonHeld(int which);
bool getButtonToggle(int which);
void clearButtonToggles();
void updateButtonLogic();

// === LED FUNCTIONS ===
void serviceIndicators();
void setIndicatorMode(int which, int state);
void setAllIndicators(int state);
void clearIndicators();
void updateIndicatorLogic();

// === SHOOTER FUNCTIONS ===
void initPulseOutput();
void triggerPulseOutput();

#endif // HARDWARE_H
