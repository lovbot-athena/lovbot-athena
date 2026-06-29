#include "core.h"

// Global variables
int drivePolarity[5];
MotionCommand driveCommand;
int targetSideMode, behaviorRole, behaviorMode, unitProfileId = 1, headingMode = 1;
bool startPhaseArmed = false;

// Motor control variables
int headingDeadbandDeg = 41, openLoopTurnPower = 9, headingControlMode = 1, driveRampMode = 0;
double driveRampStep = 4;
int TURN_POWER_FLOOR = 12, TURN_POWER_CEIL = 36;
int currentMotorPower[5] = {0}, targetMotorPower[5] = {0};
unsigned long lastRampUpdateMs = 0;

// PID variables
double headingTrimP = 0.22, headingTrimI = 0.00, headingTrimD = 0.015;
double turnLoopP = 0.29, turnLoopI = 0.00, turnLoopD = 0.00;
double headingSetpoint, headingInput, headingTrimOutput, turnLoopOutput;

PID headingTrimPID(&headingInput, &headingTrimOutput, &headingSetpoint, headingTrimP, headingTrimI, headingTrimD, DIRECT);
PID turnLoopPID(&headingInput, &turnLoopOutput, &headingSetpoint, turnLoopP, turnLoopI, turnLoopD, DIRECT);

// Capture gate detection
long lastCaptureGateMs = 0;
int captureGateCount = -1, captureGateWindowMs = 95;

void initDebugPort() { Serial.begin(115200); }

void initRuntimeState(){
  int new_motor_dir[5] = {0,-1,-1,-1,-1};
  for(int i = 0; i < 5; i++){
    drivePolarity[i] = new_motor_dir[i];
  }
}

void initDriveMotors() {
  for (int i = 1; i <= 4; i++) {
    pinMode(PWM_PIN[i], OUTPUT);
    pinMode(DIR_PIN[i], OUTPUT);
  }
  headingTrimPID.SetMode(AUTOMATIC);
  headingTrimPID.SetOutputLimits(-38, 38);
  turnLoopPID.SetMode(AUTOMATIC);
  turnLoopPID.SetOutputLimits(-38, 38);
  headingSetpoint = 0;
}

// Basic getters/setters
int getHeadingTarget() { return driveCommand.target; }
void setHeadingTarget(int newTarget) { driveCommand.target = newTarget; }
int getDrivePower() { return driveCommand.speed; }
void setDrivePower(int newSpeed) { driveCommand.speed = newSpeed; }
int getDriveHeading() { return driveCommand.direction; }
void setDriveHeading(int newDirection) { driveCommand.direction = newDirection; }
int getTargetSideMode() { return targetSideMode; }
void setTargetSideMode(int a) { targetSideMode = a; }
int getBehaviorRole() { return behaviorRole; }
void setBehaviorRole(int a) { behaviorRole = a; }
int getUnitProfileId() { return EEPROM.read(32); }
void setUnitProfileId(int a) { EEPROM.write(32, a); }

// Math utilities
int signedAngleDelta(int target, int currentValue) {
  int current = currentValue;
  if (target < 180) {
    return current > target + 180 ? -360 + current - target : current - target;
  } else {
    return current > target - 180 ? current - target : 360 - target + current;
  }
}

// Motor control
void writeDriveMotor(int which, int motorSpeed) {
  motorSpeed *= drivePolarity[which];
  motorSpeed = constrain(motorSpeed, -DRIVE_POWER_LIMIT, DRIVE_POWER_LIMIT);
  digitalWrite(DIR_PIN[which], motorSpeed >= 0 ? LOW : HIGH);
  analogWrite(PWM_PIN[which], abs(motorSpeed) * 255 / 100);
}

void writeDriveMotors(int s1, int s2, int s3, int s4) {
  writeDriveMotor(1, s1); writeDriveMotor(2, s2); writeDriveMotor(3, s3); writeDriveMotor(4, s4);
}

void applyMotionCommand(){
  int angleDif = signedAngleDelta(getHeadingTarget(), getHeadingAngle());
  if (abs(angleDif) > headingDeadbandDeg) {
    int val = angleDif < 0 ? openLoopTurnPower : -openLoopTurnPower;
    writeDriveMotors(val, val, val, val);
  }
}

// Object detection
bool captureGateActive() { return analogRead(CAPTURE_GATE_PIN) < 14; }
bool hasObjectCapture() { return captureGateActive() && (getObjectBearing() < 60 || getObjectBearing() > 300); }

// Utility functions
long systemMillis() { return millis(); }
void waitSeconds(float sec) { delay(sec * 1000); }
void stopDriveNow() { writeDriveMotors(0, 0, 0, 0); }
