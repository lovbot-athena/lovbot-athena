#ifndef OBJECT_TRACKER_H
#define OBJECT_TRACKER_H

#include <Arduino.h>
#include <PID_v2.h>
#include "camera.h"
#include "compoundEye.h"
#include "controller.h"

// ---------------------- Function Declarations ---------------------- //

// Initializes object-tracking PID controllers
void initObjectTrackerPID();

// Controls robot to approach the object using PID
void trackObjectPID();

// Resets the PID state to prevent overshooting or instability
void resetObjectTrackerPID();

#endif // OBJECT_TRACKER_H
