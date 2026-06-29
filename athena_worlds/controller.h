#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <Arduino.h>

#include "func.h"
#include "compass.h"
#include "camera.h"
#include "button.h"
#include "led.h"
#include "dribbler.h"
#include "pulse_output.h"
#include "display.h"
#include "communication.h"
#include "offense.h"
#include "defense.h"
#include "DFUltraTrig.h"
#include "grayscale.h"
#include "object_tracker.h"
#include "coordinate.h"
#include "coordinateMovement.h"
#include "config.h"


// === Main Control ===
void runMainController();
bool hasObjectCapture();

// === Dribbler and Pulse Output ===
void dribblerControl();
void pulseOutputControl();
bool shootStrategyCondition();

#endif
