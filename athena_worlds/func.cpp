#include "func.h"

double headingTrimP = 0.22, headingTrimI = 0.00, headingTrimD = 0.015;
double turnLoopP = 0.29, turnLoopI = 0.00, turnLoopD = 0.00;
double headingSetpoint, headingInput, headingTrimOutput, turnLoopOutput;

PID headingTrimPID(&headingInput, &headingTrimOutput, &headingSetpoint, headingTrimP, headingTrimI, headingTrimD, DIRECT);
PID turnLoopPID(&headingInput, &turnLoopOutput, &headingSetpoint, turnLoopP, turnLoopI, turnLoopD, DIRECT);

int headingDeadbandDeg = 41;
int openLoopTurnPower = 9;
double driveRampStep = 4;
int headingControlMode = 1;
int driveRampMode = 0;
int TURN_POWER_FLOOR = 12;
int TURN_POWER_CEIL = 36;

int currentMotorPower[5] = {0};
int targetMotorPower[5] = {0};
unsigned long lastRampUpdateMs = 0;

int targetSideMode;
int behaviorRole;
int behaviorMode;
bool startPhaseArmed = false;
int unitProfileId = 1;
int headingMode = 1;

long lastCaptureGateMs = 0;
bool captureGateSeen = false;
int captureGateTimerMs = -10000;
int captureGateCount = -1;
int captureGateWindowMs = 95;

MotionCommand driveCommand;

int motorSign[4][4] = {
  { 1,  1, -1, -1},
  {-1,  1,  1, -1},
  {-1, -1,  1,  1},
  { 1, -1, -1,  1}
};

void initDebugPort() {
  Serial.begin(115200);
}

void setDrivePolarity(int dir1, int dir2, int dir3, int dir4) {
  drivePolarity[1] = dir1;
  drivePolarity[2] = dir2;
  drivePolarity[3] = dir3;
  drivePolarity[4] = dir4;
}

void initDriveMotors() {
  Serial.println("setup motors");
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

void setHeadingDeadband(int angle) { headingDeadbandDeg = angle; }
int getHeadingDeadband() { return headingDeadbandDeg; }
void setOpenLoopTurnPower(int s) { openLoopTurnPower = s; }

int getHeadingTarget() { return driveCommand.target; }
void setHeadingTarget(int newTarget) { driveCommand.target = newTarget; }
int getDrivePower() { return driveCommand.speed; }
void setDrivePower(int newSpeed) { driveCommand.speed = newSpeed; }
int getDriveHeading() { return driveCommand.direction; }
void setDriveHeading(int newDirection) { driveCommand.direction = newDirection; }

void setHeadingControlMode(int mode) { headingControlMode = mode; }
int getHeadingControlMode() { return headingControlMode; }
void setDriveRampMode(int mode) { driveRampMode = mode; }
int getDriveRampMode() { return driveRampMode; }
void setDriveRampStep(int change) { driveRampStep = change; }
double getDriveRampStep() { return driveRampStep; }

void applyMotionCommand(){
  int d = driveCommand.direction;
  int s = driveCommand.speed;
  int targetAngle = driveCommand.target;

  int angleDif = signedAngleDelta(getHeadingTarget(), getHeadingAngle());
  headingInput = angleDif;
  headingTrimPID.Compute();
  turnLoopPID.Compute();

  int speed1 = 0, speed2 = 0, speed3 = 0, speed4 = 0;

  if (headingControlMode == MODE_DIRECT) {
    if (abs(angleDif) > headingDeadbandDeg) {
      int val = angleDif < 0 ? openLoopTurnPower : -openLoopTurnPower;
      speed1 = val; speed2 = val; speed3 = val; speed4 = val;
    }
  } else {
    if (abs(angleDif) > headingDeadbandDeg) {
      int val = angleDif < 0
        ? constrain(turnLoopOutput, TURN_POWER_FLOOR, TURN_POWER_CEIL)
        : constrain(turnLoopOutput, -TURN_POWER_CEIL, -TURN_POWER_FLOOR);
      speed1 = val; speed2 = val; speed3 = val; speed4 = val;
    }
  }

  if (driveRampMode == 0) {
    writeDriveMotors(speed1, speed2, speed3, speed4);
  } else {
    rampDriveMotors(speed1, speed2, speed3, speed4);
  }
}

double degToRadLocal(int degree) { return degree * 3.1415926 / 180; }
double radToDegLocal(double radian) { return radian * 180 / 3.14159265; }

int scaledSineProjection(int degree1, int degree2, int speed2) {
  double rad1 = degToRadLocal(degree1);
  double rad2 = degToRadLocal(degree2);
  return (sin(rad1) * speed2) / sin(rad2);
}

int signedAngleDelta(int target, int currentValue) {
  int current = currentValue;
  if (target < 180) {
    return current > target + 180 ? -360 + current - target : current - target;
  } else {
    return current > target - 180 ? current - target : 360 - target + current;
  }
}

int foldedAngleMagnitude(int a) {
  a = abs(a) % 360;
  return a > 180 ? 360 - a : a;
}

void writeDriveMotor(int which, int motorSpeed) {
  motorSpeed *= drivePolarity[which];
  motorSpeed = constrain(motorSpeed, -DRIVE_POWER_LIMIT, DRIVE_POWER_LIMIT);

  digitalWrite(DIR_PIN[which], motorSpeed >= 0 ? LOW : HIGH);
  analogWrite(PWM_PIN[which], motorSpeed >= 0 || driveSignalMode[which] == 1 ? abs(motorSpeed) * 255 / 100 : 255 + motorSpeed * 255 / 100);
}

void writeDriveMotors(int s1, int s2, int s3, int s4) {
  writeDriveMotor(1, s1);
  writeDriveMotor(2, s2);
  writeDriveMotor(3, s3);
  writeDriveMotor(4, s4);
}

void rampDriveMotors(int s1, int s2, int s3, int s4) {
  targetMotorPower[1] = s1;
  targetMotorPower[2] = s2;
  targetMotorPower[3] = s3;
  targetMotorPower[4] = s4;

  if (millis() - lastRampUpdateMs > 1) {
    for (int i = 1; i <= 4; i++) {
      if (abs(targetMotorPower[i] - currentMotorPower[i]) < driveRampStep) {
        currentMotorPower[i] = targetMotorPower[i];
      } else {
        currentMotorPower[i] += (targetMotorPower[i] - currentMotorPower[i]) / abs(targetMotorPower[i] - currentMotorPower[i]) * driveRampStep;
      }
    }
    lastRampUpdateMs = millis();
  }

  writeDriveMotors(currentMotorPower[1], currentMotorPower[2], currentMotorPower[3], currentMotorPower[4]);
}

void stopDriveNow() {
  writeDriveMotors(0, 0, 0, 0);
}

bool captureGateActive() {
  return analogRead(CAPTURE_GATE_PIN) < 14;
}

bool hasObjectCapture() {
  if (captureGateActive() && (getObjectBearing() < 60 || getObjectBearing() > 300)) {
    captureGateCount++;
    lastCaptureGateMs = millis();
    return true;
  } else {
    if (millis() - lastCaptureGateMs < 72) {
      return true;
    }
    captureGateCount = -1;
    return false;
  }
}

long systemMillis() { return millis(); }
void resetSystemClockStub() {}
void waitSeconds(float sec) { delay(sec * 1000); }

void setTargetSideMode(int a) { targetSideMode = a; }
int getTargetSideMode() { return targetSideMode; }

void setBehaviorRole(int a) { behaviorRole = a; }
int getBehaviorRole() { return behaviorRole; }

void setBehaviorMode(int a) { behaviorMode = a; }
int getBehaviorMode() { return behaviorMode; }

int getHeadingMode() { return headingMode; }
void setHeadingMode(int a) { headingMode = a; }

bool getStartPhaseArmed() { return startPhaseArmed; }
void setStartPhaseArmed(bool a) { startPhaseArmed = a; }

void setUnitProfileId(int a) { EEPROM.write(32, a); }
int getUnitProfileId() { return EEPROM.read(32); }
