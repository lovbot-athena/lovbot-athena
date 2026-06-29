#include "object_tracker.h"

int objectTrackMinPower = 18;
int objectTrackMaxPower = 49;
int objectTrackPowerSpan = objectTrackMaxPower - objectTrackMinPower;

int offsetMinDeg = 0;
int offsetMaxDeg = 76;
int offsetSpanDeg = offsetMaxDeg - offsetMinDeg;

double nearObjectSetpoint, farObjectSetpoint, objectSignal, rangePowerRatio, bearingOffsetRatio;
double rangeLoopP = 0.92, rangeLoopI = 0.000, rangeLoopD = 0.000;
double offsetLoopP = 84.0, offsetLoopI = 0.00, offsetLoopD = 0.018;

double PID_PUBLIC_MIN = 0;
double PID_PUBLIC_MAX = 92;

PID rangePowerPID(&objectSignal, &rangePowerRatio, &nearObjectSetpoint, rangeLoopP, rangeLoopI, rangeLoopD, DIRECT);
PID bearingOffsetPID(&objectSignal, &bearingOffsetRatio, &farObjectSetpoint, offsetLoopP, offsetLoopI, offsetLoopD, REVERSE);

void initObjectTrackerPID() {
    if (getUnitProfileId == 0) {
        nearObjectSetpoint = 224;
    } else {
        nearObjectSetpoint = 224;
    }
    farObjectSetpoint = 27;

    rangePowerPID.SetOutputLimits(PID_PUBLIC_MIN, PID_PUBLIC_MAX);
    rangePowerPID.SetMode(AUTOMATIC);
    bearingOffsetPID.SetOutputLimits(PID_PUBLIC_MIN, PID_PUBLIC_MAX);
    bearingOffsetPID.SetMode(AUTOMATIC);
}

void trackObjectPID() {
    int objectBearing = getObjectBearing();
    objectSignal = getObjectSignalFiltered();

    bearingOffsetPID.Compute();
    rangePowerPID.Compute();

    double publicDrivePower = objectTrackMaxPower * (rangePowerRatio / 100.0);
    publicDrivePower = constrain(publicDrivePower, objectTrackMinPower, objectTrackMaxPower);

    double bearingBlendRatio = objectBearing > 180 ? (360.0 - objectBearing) / 180.0 : objectBearing / 180.0;
    bearingBlendRatio = constrain(bearingBlendRatio, 0.0, 1.0);
    double k = 3.25;
    bearingBlendRatio = 1.0 - exp(-k * bearingBlendRatio);

    double offset = offsetSpanDeg * (bearingOffsetRatio / 100.0) * bearingBlendRatio;
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
    rangePowerPID.SetOutputLimits(0.0, 1.0);
    rangePowerPID.SetOutputLimits(-1.0, 0.0);
    rangePowerPID.SetOutputLimits(PID_PUBLIC_MIN, PID_PUBLIC_MAX);

    bearingOffsetPID.SetOutputLimits(0.0, 1.0);
    bearingOffsetPID.SetOutputLimits(-1.0, 0.0);
    bearingOffsetPID.SetOutputLimits(PID_PUBLIC_MIN, PID_PUBLIC_MAX);
}
