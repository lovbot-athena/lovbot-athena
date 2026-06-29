#ifndef FUNC_H
#define FUNC_H

struct FieldPose { int posX; int posY; bool isXBlocked; bool isYBlocked; long timestamp; };
struct PoseSample { int posX; int posY; unsigned long timeX; unsigned long timeY; };
struct MotionPacket { int dir; int dist; };
struct MotionCommand { int speed; int direction; int target; };
struct UnitProfileA {int drivePolarity[4] = {0,-1,-1,-1}; int nearObjectSetpoint = 750;};
struct UnitProfileB {int drivePolarity[4] = {0,-1,-1,-1}; int nearObjectSetpoint = 750;};
struct UnitProfileActive {int drivePolarity[4]; int nearObjectSetpoint;};
static PoseSample lastPoseSample;
static FieldPose activePose;

#include <Arduino.h>
#include <Wire.h>
#include "compass.h"
#include "button.h"
#include "led.h"
#include "pulse_output.h"
#include <PID_v2.h>
#include "compoundEye.h"
#include "DFUltraTrig.h"
#include "config.h"

// -------------------------- Constants & Macros --------------------------

#define VECTOR_TRIM_RATIO 0.31

#define OFF     0
#define ON      1
#define BLINK   2

#define SIDE_A  1
#define SIDE_B    0

#define ROLE_PRIMARY   1
#define ROLE_SECONDARY   2

#define ROLE_PRIMARY_SWITCH  1
#define ROLE_SECONDARY_SWITCH  2
#define ROLE_AUTO_SWITCH  3
#define ROLE_PRIMARY_LOCK     4
#define ROLE_SECONDARY_LOCK     5

#define JM            1
#define SMALL_ULTRA   1
#define INFRARED      2
#define CM            3
#define LARGE_ULTRA   4
#define BNO055        5

#define DIR_FRONT   0
#define DIR_RIGHT   1
#define DIR_BACK    2
#define DIR_LEFT    3

#define DRIVE_POWER_LIMIT  92
#define MOVE_STOP_CODE       360   // Representation of stop in applyMotionCommand()
#define MOVE_BLOCKED_CODE    361   // Representation of blocked in applyMotionCommand()

#define FIELD_X_SCALE 280  // Horizontal field size (new robot)
#define FIELD_Y_SCALE 320  // Vertical field size

const int xSoftLimitMargin = 92;
const int ySoftLimitMargin = 134;//40
const int xSoftLimit = FIELD_X_SCALE-xSoftLimitMargin; 
const int ySoftLimit = FIELD_Y_SCALE-ySoftLimitMargin; 

// -------------------------- Pin Configuration --------------------------

static const int CAPTURE_GATE_PIN       = A6;
static const int PULSE_OUT_PIN       = A12;
static const int USER_BUTTON_PINS[]    = {10, 11, 12, 30, 31};
// static const int DRIBBLER_PIN_1  = A9;

#define TCAADDR 0x70  // I2C Multiplexer Address

// -------------------------- Motor Configuration --------------------------

static int PWM_PIN[5]       = {0, 6, 7, 8, 9};   // PWM_PIN pins for motors 1–4
static int DIR_PIN[5]       = {0, 2, 3, 4, 5};   // Direction pins for motors 1–4
static int driveSignalMode[5]= {0, 1, 1, 1, 1};   // Motor signal polarity correction

// -------------------------- Function Prototypes --------------------------

// Setup
void initDebugPort();
void setDrivePolarity(int dir1, int dir2, int dir3, int dir4);
void initDriveMotors();
void setEyeType(int type);
void setHeadingDeadband(int angle);
int getHeadingDeadband();
void setOpenLoopTurnPower(int s);
void setHeadingControlMode(int mode);
int getHeadingControlMode();
void setDriveRampMode(int mode);
int getDriveRampMode();
void setDriveRampStep(int change);
double getDriveRampStep();

// Motion
int getHeadingTarget();
void setHeadingTarget(int newTarget);
int getDrivePower();
void setDrivePower(int newSpeed);
int getDriveHeading();
void setDriveHeading(int newDirection);

void applyMotionCommand();
int scaledSineProjection(int degree1, int degree2, int speed2);
int signedAngleDelta(int target, int currentValue);
int foldedAngleMagnitude(int a);
double degToRadLocal(int degree);
double radToDegLocal(double radian);

void writeDriveMotor(int which, int motorSpeed);
void writeDriveMotors(int speed1, int speed2, int speed3, int speed4);
void rampDriveMotors(int speed1, int speed2, int speed3, int speed4);
void stopDriveNow();

// Capture gate / Object
void setPulseOutput(int state);
bool captureGateActive();
bool hasObjectCapture();

// Time / Utility
long systemMillis();
void resetSystemClockStub();
void waitSeconds(float sec);

void setVectorTrimRatio(double ratio);

// Robot State
void setTargetSideMode(int a);
int getTargetSideMode();

void setBehaviorRole(int a);
int getBehaviorRole();

void setBehaviorMode(int a);
int getBehaviorMode();

void setUnitProfileId(int a);
int getUnitProfileId();

int getHeadingMode();
void setHeadingMode(int a);

void setStartPhaseArmed(bool a);
bool getStartPhaseArmed();

#endif // FUNC_H