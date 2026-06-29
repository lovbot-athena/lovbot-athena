#ifndef LED_H
#define LED_H

#include <Arduino.h>
#include "func.h"

// Initializes or updates the LED outputs based on current state
void serviceIndicators();

// Sets the state (OFF, ON, BLINK) of a specific LED
void setIndicatorMode(int which, int state);

// Turns off all LEDs
void clearIndicators();

// Sets all LEDs to a specific state (OFF, ON, BLINK)
void setAllIndicators(int state);

// Updates LEDs based on game state (e.g., startPhaseArmed or line detected)
void updateIndicatorLogic();

#endif // LED_H
