#include "sensors.h"

// === CAMERA VARIABLES ===
byte visionFrame[8];

// === COMPASS VARIABLES ===
Adafruit_BNO055 orientationSensor = Adafruit_BNO055(61);
int headingReferenceDeg = 4;

// === COMPOUND EYE VARIABLES ===
const int objectSectorCount = 18;
int objectSectorThreshold[objectSectorCount] = { 28, 61, 93, 124, 157, 188, 219, 251, 282, 314, 346, 376, 407, 439, 470, 502, 532, 563};

// === GRAYSCALE VARIABLES ===
int analogSensePins[] = {A0, A1};
int muxSelectPins[] = {38, 39, 40, 41};
const int ringSensorCount = 32;
int ringSensorValue[ringSensorCount];
int ringEdgeThreshold[ringSensorCount];
bool ringEdgeActive[ringSensorCount];
bool boundaryActive = false;

// === ULTRASONIC VARIABLES ===
int rangeAnalogPin[RANGE_SENSOR_COUNT] = {A9, A13, A7, A8};

// === CAMERA FUNCTIONS ===
void initVisionPort() {
  Serial7.begin(115200);
}

void updateVisionFrame() {
  if (Serial7.available() >= 10) {
    byte header = Serial7.read();
    if (header == 0xA3) {
      for (int i = 0; i < 8; i++) {
        visionFrame[i] = Serial7.read();
      }
      Serial7.read(); // checksum
    }
  }
}

bool localTargetVisible() { return getTargetSideMode() == 0 ? (visionFrame[2] > 0) : (visionFrame[7] > 0); }
int getLocalTargetAngle() { return getTargetSideMode() == 0 ? visionFrame[0] * 2 : visionFrame[5] * 2; }
int getLocalTargetRange() { return getTargetSideMode() == 0 ? visionFrame[1] : visionFrame[6]; }
bool remoteTargetVisible() { return getTargetSideMode() == 1 ? (visionFrame[2] > 0) : (visionFrame[7] > 0); }
int getRemoteTargetAngle() { return getTargetSideMode() == 1 ? visionFrame[0] * 2 : visionFrame[5] * 2; }
int getRemoteTargetRange() { return getTargetSideMode() == 1 ? visionFrame[1] : visionFrame[6]; }

// === COMPASS FUNCTIONS ===
int getHeadingAngle() {
  int rawDegree = orientationSensor.getVector(Adafruit_BNO055::VECTOR_EULER).x();
  return (rawDegree - headingReferenceDeg + 360) % 360;
}

void initHeadingSensor() {
  if (!orientationSensor.begin()) {
    while (1);
  }
  orientationSensor.setExtCrystalUse(true);
  orientationSensor.setMode(0x0C);
}

// === COMPOUND EYE FUNCTIONS ===
void initObjectDirectionSensor() {
  pinMode(OBJECT_BEARING_PIN, INPUT);
  pinMode(OBJECT_SIGNAL_PIN, INPUT);
}

int getObjectBearing() {
  int sectorRawValue = analogRead(OBJECT_BEARING_PIN);
  for (int i = 0; i < objectSectorCount - 1; i++) {
    if (sectorRawValue < (objectSectorThreshold[i] + objectSectorThreshold[i + 1]) / 2) {
      return i * (360.0 / objectSectorCount);
    }
  }
  return (objectSectorCount - 1) * (360.0 / objectSectorCount);
}

int getObjectSignal() {
  return analogRead(OBJECT_SIGNAL_PIN);
}

// === GRAYSCALE FUNCTIONS ===
void initBoundarySensors() {
  for (int i = 0; i < 4; i++) pinMode(muxSelectPins[i], OUTPUT);
  for (int i = 0; i < 2; i++) pinMode(analogSensePins[i], INPUT);
  for (int i = 0; i < 32; i++) {
    ringEdgeThreshold[i] = EEPROM.read(i) * 2;
  }
}

void sampleBoundarySensors() {
  for (int i = 0; i < ringSensorCount / 2; i++) {
    for (int j = 0; j < 4; j++) {
      digitalWrite(muxSelectPins[j], (i >> j) & 1);
    }
    ringSensorValue[i] = analogRead(analogSensePins[0]);
    ringSensorValue[i + 16] = analogRead(analogSensePins[1]);
  }
  
  boundaryActive = false;
  for (int i = 0; i < ringSensorCount; i++) {
    ringEdgeActive[i] = ringSensorValue[i] > ringEdgeThreshold[i];
    if (ringEdgeActive[i]) boundaryActive = true;
  }
}

void updateBoundaryVector() {}
void updateBoundaryVectorPrimary() {}
bool boundaryDetected() { return boundaryActive; }

// === ULTRASONIC FUNCTIONS ===
void sampleRangeInputs() {}

int getRangeFront() { return analogRead(rangeAnalogPin[0]); }
int getRangeRight() { return analogRead(rangeAnalogPin[1]); }
int getRangeBack() { return analogRead(rangeAnalogPin[2]); }
int getRangeLeft() { return analogRead(rangeAnalogPin[3]); }
