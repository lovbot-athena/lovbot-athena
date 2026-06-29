#ifndef GRAYSCALE_H
#define GRAYSCALE_H

#include <Arduino.h>
#include <queue>
#include "func.h"
#include <EEPROM.h>

// ------------------------------
// Initialization & Calibration
// ------------------------------
void initBoundarySensors();
void calibrateBoundaryThreshold();
void resetBoundaryThresholds();

// EEPROM Storage
void saveBoundaryThresholdsToEEPROM();
bool loadBoundaryThresholdsFromEEPROM();
void resetEEPROM(); 

// ------------------------------
// Sensor Data Processing
// ------------------------------
void sampleBoundarySensors();
void updateBoundaryVector();
int getBoundaryAngleDelta(int currentLight, int lastLight);


// ------------------------------
// Boundary Detection
// ------------------------------
bool boundaryDetected();
void resetBoundaryLatch();
int getBoundaryLatchCount();

void loadBoundaryThresholdFromEEPROM();

void getBoundarySensorMask(bool output[32]);

int getCalibrationBlendPercent();


#endif // GRAYSCALE_H
