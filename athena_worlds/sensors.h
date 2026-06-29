#ifndef SENSORS_H
#define SENSORS_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <PID_v2.h>
#include <EEPROM.h>
#include <queue>
#include "core.h"

// === COMPASS CONSTANTS ===
#define RST              11    
#define BNO_ADDR         0x18  
#define PAGE_ID          0x07  
#define ACC_DATA_X_LSB   0x08  
#define OPR_MODE         0x3D  
#define ACC_CONFIG       0x08  
#define MODE_AMG         0x07  

// === COMPOUND EYE CONSTANTS ===
#define OBJECT_SIGNAL_FLOOR      46  
#define OBJECT_BEARING_PIN    A10 
#define OBJECT_SIGNAL_PIN   A11 

// === ULTRASONIC CONSTANTS ===
#define RANGE_SENSOR_COUNT 4
#define RANGE_SOUND_SPEED(temp) ((331.5 + 0.6 * (float)(temp)) * 100 / 1000000.0)

// === CAMERA FUNCTIONS ===
void initVisionPort();
void updateVisionFrame();
bool localTargetVisible();
int getLocalTargetAngle();
int getLocalTargetRange();
bool remoteTargetVisible();
int getRemoteTargetAngle();
int getRemoteTargetRange();

// === COMPASS FUNCTIONS ===
void initHeadingSensor();
int getHeadingAngle();
int getHeadingDelta(int target, int currentValue);

// === COMPOUND EYE FUNCTIONS ===
void initObjectDirectionSensor();
int getObjectBearing();
int getObjectSignal();

// === GRAYSCALE FUNCTIONS ===
void initBoundarySensors();
void sampleBoundarySensors();
void updateBoundaryVector();
void updateBoundaryVectorPrimary();
bool boundaryDetected();

// === ULTRASONIC FUNCTIONS ===
void sampleRangeInputs();
int getRangeFront();
int getRangeRight();
int getRangeBack();
int getRangeLeft();

#endif // SENSORS_H
