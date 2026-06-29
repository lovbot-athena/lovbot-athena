#ifndef OFFENSE_H
#define OFFENSE_H

#include "func.h"
#include "compass.h"
#include "camera.h"
#include "button.h"
#include "led.h"
#include "dribbler.h"
#include "pulse_output.h"
#include "display.h"
#include "communication.h"
#include "grayscale.h"
#include "object_tracker.h"
#include "defense.h"
#include "controller.h"

// === Main Offense Control ===
void runPrimaryBehavior();
void runCaptureTurn();


bool inBoundaryPocket();


#endif
