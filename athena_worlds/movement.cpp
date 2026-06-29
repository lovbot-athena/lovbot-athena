#include "movement.h"

// === BALL TRACKING VARIABLES ===
int objectTrackMinPower = 18;
int objectTrackMaxPower = 52;
double nearObjectSetpoint = 214;
double farObjectSetpoint = 26;
double objectSignal, rangePowerRatio, bearingOffsetRatio;
double rangeLoopP = 0.96, rangeLoopI = 0.000, rangeLoopD = 0.000;
double offsetLoopP = 78.0, offsetLoopI = 0.00, offsetLoopD = 0.021;
PID rangePowerPID(&objectSignal, &rangePowerRatio, &nearObjectSetpoint, rangeLoopP, rangeLoopI, rangeLoopD, DIRECT);
PID bearingOffsetPID(&objectSignal, &bearingOffsetRatio, &farObjectSetpoint, offsetLoopP, offsetLoopI, offsetLoopD, REVERSE);

// === COORDINATE FUNCTIONS ===
FieldPose estimateFieldPose(int front, int right, int back, int left) {
  FieldPose estimatedPose;
  estimatedPose.posX = (left - right) / 2;
  estimatedPose.posY = (back - front) / 2;
  return estimatedPose;
}

// === BALL TRACKING FUNCTIONS ===
void initObjectTrackerPID() {
  rangePowerPID.SetOutputLimits(0, 92);
  rangePowerPID.SetMode(AUTOMATIC);
  bearingOffsetPID.SetOutputLimits(0, 92);
  bearingOffsetPID.SetMode(AUTOMATIC);
}

void trackObjectPID() {
  int objectBearing = getObjectBearing();
  objectSignal = getObjectSignal();
  
  bearingOffsetPID.Compute();
  rangePowerPID.Compute();
  
  double publicDrivePower = objectTrackMaxPower * (rangePowerRatio / 100.0);
  publicDrivePower = constrain(publicDrivePower, objectTrackMinPower, objectTrackMaxPower);
  
  double bearingBlendRatio = objectBearing > 180 ? (360.0 - objectBearing) / 180.0 : objectBearing / 180.0;
  bearingBlendRatio = constrain(bearingBlendRatio, 0.0, 1.0);
  
  double offset = 74 * (bearingOffsetRatio / 92.0) * bearingBlendRatio;
  if (objectBearing >= 180) {
    offset *= -1;
  }
  
  double publicDriveHeading = objectBearing + offset;
  setDriveHeading(publicDriveHeading);
  setDrivePower(publicDrivePower);
  
  if (objectSignal < 14) {
    resetObjectTrackerPID();
  }
}

void resetObjectTrackerPID() {
  rangePowerPID.SetOutputLimits(0, 92);
  bearingOffsetPID.SetOutputLimits(0, 92);
}
