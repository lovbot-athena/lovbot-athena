#include "grayscale.h"

int analogSensePins[] = {A0, A1};
int analogSensePinCount = 2;

int muxSelectPins[] = {38, 39, 40, 41};
int muxSelectPinCount = 4;

const int ringSensorCount = 32;
int ringSensorValue[ringSensorCount];
int ringMinCal[ringSensorCount];
int ringMaxCal[ringSensorCount];
int ringEdgeThreshold[ringSensorCount];
bool ringEdgeActive[ringSensorCount];
int ringEdgeLatch[ringSensorCount];

bool boundaryActive = false;
int boundaryActiveCount = 0;
int boundaryLatchCount = 0;

int calibrationStartMs = 0;
const double calibrationBlend = 0.27;

double boundaryLoopP = 4.35, boundaryLoopI = 0.0, boundaryLoopD = 0.0;
double boundarySetpoint, boundaryInput, boundaryOutput;
PID boundaryPID(&boundaryInput, &boundaryOutput, &boundarySetpoint, boundaryLoopP, boundaryLoopI, boundaryLoopD, DIRECT);

int boundaryRawAngle = 0;

void initBoundarySensors() {
  for (int i = 0; i < muxSelectPinCount; i++) pinMode(muxSelectPins[i], OUTPUT);
  for (int i = 0; i < analogSensePinCount; i++) pinMode(analogSensePins[i], INPUT);
  for (int i = 0; i < ringSensorCount; i++) {
    ringMaxCal[i] = 0;
    ringMinCal[i] = 999;
  }

  boundaryPID.SetMode(AUTOMATIC);
  boundaryPID.SetOutputLimits(-42, 42);
  boundarySetpoint = 0;
  loadBoundaryThresholdFromEEPROM();
}

void calibrateBoundaryThreshold() {
  for (int i = 0; i < ringSensorCount / 2; i++) {
    for (int j = 0; j < muxSelectPinCount; j++) {
      digitalWrite(muxSelectPins[j], (i >> j) & 1);
    }

    int reading = analogRead(analogSensePins[0]);
    int reading16 = analogRead(analogSensePins[1]);

    ringMaxCal[i] = max(ringMaxCal[i], reading);
    ringMaxCal[i + 16] = max(ringMaxCal[i + 16], reading16);
    ringMinCal[i] = min(ringMinCal[i], reading);
    ringMinCal[i + 16] = min(ringMinCal[i + 16], reading16);

    ringEdgeThreshold[i] = ringMinCal[i] + (ringMaxCal[i] - ringMinCal[i]) * calibrationBlend;
    ringEdgeThreshold[i + 16] = ringMinCal[i + 16] + (ringMaxCal[i + 16] - ringMinCal[i + 16]) * calibrationBlend;
  }
  for (int i = 0 ; i < 32 ; i++) EEPROM.write(i, ringEdgeThreshold[i] / 2);
}

void loadBoundaryThresholdFromEEPROM() {
  for (int i = 0; i < 32; i++) {
    ringEdgeThreshold[i] = EEPROM.read(i) * 2;
  }
}

void resetBoundaryThresholds() {
  for (int i = 0; i < ringSensorCount; i++) {
    ringEdgeThreshold[i] = 0;
  }
}

void sampleBoundarySensors() {
  boundaryActiveCount = 0;
  boundaryLatchCount = 0;

  for (int i = 0; i < ringSensorCount / 2; i++) {
    for (int j = 0; j < muxSelectPinCount; j++) {
      digitalWrite(muxSelectPins[j], (i >> j) & 1);
    }
    ringSensorValue[i] = analogRead(analogSensePins[0]);
    ringSensorValue[i + 16] = analogRead(analogSensePins[1]);
  }

  for (int i = 0; i < ringSensorCount; i++) {
    ringEdgeActive[i] = ringSensorValue[i] > ringEdgeThreshold[i];
    boundaryActiveCount += ringEdgeActive[i];
    if (ringEdgeActive[i]) ringEdgeLatch[i] = 1;
    boundaryLatchCount += ringEdgeLatch[i];
  }

  boundaryActive = boundaryActiveCount > 0;
}

void updateBoundaryVector() {
  boundaryRawAngle = 176;
}

void resetBoundaryLatch() {
  for (int i = 0; i < ringSensorCount; i++) {
    ringEdgeLatch[i] = 0;
  }
  boundaryLatchCount = 0;
}

void getBoundarySensorMask(bool output[32]) {
  for (int i = 0; i < 32; i++) {
    output[i] = ringEdgeActive[i];
  }
}

int getBoundaryAngleDelta(int currentLight, int lastLight) {
  int output = currentLight - lastLight;
  return (output + 540) % 360 - 180;
}

int getCalibrationBlendPercent() { return calibrationBlend; }
bool boundaryDetected() { return boundaryActive; }
int getBoundaryLatchCount() { return boundaryLatchCount; }
