#include <Arduino.h>
#include "core.h"
#include "sensors.h"
#include "hardware.h"
#include "communication.h"
#include "movement.h"
#include "controller.h"

void setup() {
  initDebugPort(); 
  Serial.print("Boot");
  initButtons();
  setIndicatorMode(4, 1);
  Serial.println("Starting controller setup...");
  initDriveMotors();
  Serial.println("Starting sensor setup...");
  initPulseOutput();
  initVisionPort();
  initHeadingSensor();
  initPeerLink();
  initBoundarySensors();
  initObjectDirectionSensor();
  initObjectTrackerPID();

  Serial.println("Setup complete.");
}

void loop() {
  runMainController();
}
