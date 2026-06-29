#include "controller.h"


void runMainController() {
  Serial.print("Player");
  // === Initialization ===
  setTargetSideMode(SIDE_B);     // Set goal to shoot on
  setBehaviorRole(ROLE_PRIMARY);     // Set initial role
  initRuntimeState();
  // while(1){
  //   writeDriveMotors(20,20,-20,-20);
  // }

  while (1) {
    // === Core Sensor & Input Processing ===
    sampleButtons();
    displayUpdate();
    updateVisionFrame();
    sampleBoundarySensors();
    getCoordinate();
    // flushPeerFrame();
    // pollPeerFrame();

    serviceIndicators();
    updateButtonLogic();
    updateIndicatorLogic();

    // === Role Handling ===
    if (getBehaviorRole() == ROLE_PRIMARY) {
      runPrimaryBehavior();
    } else {
      defenseMain();
    }

    //goToCoordinate(0, 0);
    // === Actuation ===

    applyMotionCommand();
    // writeDriveMotors(99,99,99,99);

    // === Debugging ===
    // Serial.print(" Dir: ");
    // Serial.print(getDriveHeading());
    // Serial.print(" Speed: ");
    // Serial.print(getDrivePower());

    // Serial.println(getRxSlot(0));

  }
}
