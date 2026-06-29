#ifndef PULSE_OUTPUT_H
#define PULSE_OUTPUT_H

#include "func.h"

// === Pulse Output State Definitions ===
#define ON  1
#define OFF 0

// === Pulse Output Control Functions ===
void initPulseOutput();
void triggerPulseOutput();
void printPulseOutputState();

#endif
