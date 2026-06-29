#ifndef MOVEMENT_H
#define MOVEMENT_H

#include <Arduino.h>
#include <PID_v2.h>
#include "core.h"
#include "sensors.h"

// === COORDINATE FUNCTIONS ===
FieldPose estimateFieldPose(int front, int right, int back, int left);

// === BALL TRACKING FUNCTIONS ===
void initObjectTrackerPID();
void trackObjectPID();
void resetObjectTrackerPID();

#endif // MOVEMENT_H
