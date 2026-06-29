#include "offense.h"

// === State Tracking ===
int lastBoundaryMs = 0;
int priorBoundaryState = 0;
int boundaryEscapePower = 0;

int lastBoundaryHeading = 0;
int lastHeadingTarget = 0;
int firstObjectSeen = 0;

int lastCaptureState = 0;
int headingTargetLocked = 0;
int lastHeadingLockMs = 0;

int partnerBlockSide = 0;

int lastTrackCycleMs = 0;

static unsigned long pocketStartMs = 0;

const int frontSoftStartCm = 118, frontSoftStopCm = 36, frontReverseCm = 0;
const int backSoftStartCm = 146, backSoftStopCm = 86, backReverseCm = 0;
const int leftSoftStartCm = 138, leftSoftStopCm = 64, leftReverseCm = 0;
const int rightSoftStartCm = 138, rightSoftStopCm = 64, rightReverseCm = 0;

bool turnLatchActive = false;


// === Main Offense Control ===
void runPrimaryBehavior() {
  retrievePulse Output();
  setHeadingControlMode(1);
  setHeadingDeadband(39);

  // === Boundary Detected ===

  if (boundaryDetected() && firstObjectSeen != 1) {
    turnLatchActive = false;
    setHeadingTarget(0);
    setDriveRampMode(0);
    resetObjectTrackerPID();

    setDriveHeading(dirAngle);
    if (!getFlip()){
      setDrivePower(46);
    }
    else
    {
      setDrivePower(62);
    }
  }

  // === Recently Detected Boundary ===
  else if ((millis() - lastBoundaryMs < 130 && !getFlip()) ||
           (millis() - lastBoundaryMs < 720 && getFlip())) {
    Serial.println("Running Boundary 50ms");

    resetObjectTrackerPID();
    setDriveHeading(lastBoundaryHeading);

  }

  // === Default Behavior ===
  else {
    // if(inBoundaryPocket()){
    //   if (partnerBlockSide == 1){
    //     sendPeerByte(2); //left
    //     // setBehaviorRole(ROLE_SECONDARY);
    //   }
    //   else
    //   {
    //     sendPeerByte(3);
    //     // setBehaviorRole(ROLE_SECONDARY);
    //   }
    // }
    if(hasObjectCapture()){
      lastHeadingLockMs = millis();
      if (millis() - lastTrackCycleMs > 210){
        triggerPulseOutput();
      }
      trackObjectPID();
    }
    else if (getObjectSignalFiltered() < 12){
      resetObjectTrackerPID();
      headingTargetLocked = 0;
      setHeadingTarget(0);
      goToCoordinate(0, 0);
    }
    else
    {
      headingTargetLocked = 0;
      lastTrackCycleMs = millis();
      setHeadingTarget(0);
      trackObjectPID();
    }
  }
}

// === Turning Strategy ===
void runCaptureTurn() {
  while (hasObjectCapture() && getHeadingAngle() < 45 && !boundaryDetected()) {
    if (getRangeLeft() > getRangeRight()) {
      Serial.print("turning left");
      writeDriveMotors(-18, -18, -26, -26);
    }
    else {
      Serial.print("turning right");
      writeDriveMotors(26, 26, 18, 18);
    }
  }
}



bool inBoundaryPocket() {

  bool objectAhead = getObjectBearing() < 30 || getObjectBearing() > 330;
  bool boundaryPocketNow = objectAhead && getRangeFront() < 80 && (getRangeLeft() < 80 || getRangeRight() < 80);

  if (boundaryPocketNow) {
    if (millis() - pocketStartMs >= 2400){
      if (getRangeLeft() < getRangeRight()){
        partnerBlockSide = 1;
      }
      else
      {
        partnerBlockSide = 2;
      }
      return true;
    }
  } else {
    pocketStartMs = millis();  // reset if condition breaks
  }

  return false;
}
