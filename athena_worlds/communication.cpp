#include "communication.h"

// === PEER LINK VARIABLES ===
const int PACKET_SLOT_COUNT = 4;
int lastRxSampleMs = 0;
int lastTxMs = 0;
int lastRxByte = -2;
long lastPacketMs = -12000;
int txSlots[PACKET_SLOT_COUNT];
int rxSlots[PACKET_SLOT_COUNT];

// === PEER LINK FUNCTIONS ===
void initPeerLink() {
  Serial.print("Setting up bluetooth... ");
  for (int i = 0; i < PACKET_SLOT_COUNT; i++) {
    rxSlots[i] = -1;
    txSlots[i] = -1;
  }
  Serial1.begin(19200);
  Serial.println("Success!");
}

void flushPeerFrame() {
  Serial1.write(0x9D); // Header
  int sum = 0;
  
  for (int i = 0; i < PACKET_SLOT_COUNT; i++) {
    sum += txSlots[i];
    Serial1.write(txSlots[i]);
  }
  
  int checksum = sum % 256;
  Serial1.write(checksum);
  lastTxMs = millis();
  
  for (int i = 0; i < PACKET_SLOT_COUNT; i++) {
    txSlots[i] = -1;
  }
}

void sendPeerByte(int data) {
  if (millis() - lastTxMs > 125) {
    Serial1.write(data);
    lastTxMs = millis();
  }
}

void pollPeerFrame() {
  int scratchSlots[PACKET_SLOT_COUNT];
  
  if (Serial1.available() >= PACKET_SLOT_COUNT + 2) {
    byte header = Serial1.read();
    
    if (header == 0x9D) {
      byte checksum = 0;
      
      for (int i = 0; i < PACKET_SLOT_COUNT; i++) {
        scratchSlots[i] = Serial1.read();
        checksum += scratchSlots[i];
      }
      
      byte rxChecksum = Serial1.read();
      if (rxChecksum == (checksum % 256)) {
        for (int i = 0; i < PACKET_SLOT_COUNT; i++) {
          rxSlots[i] = scratchSlots[i];
        }
        lastPacketMs = millis();
      }
    }
  } else {
    if (millis() - lastPacketMs > 24) {
      for (int i = 0; i < PACKET_SLOT_COUNT; i++) {
        rxSlots[i] = -1;
      }
    }
  }
}

int readPeerByte() {
  if (Serial1.available()) {
    lastRxByte = Serial1.read();
    lastRxSampleMs = millis();
    return lastRxByte;
  }
  
  if (millis() - lastRxSampleMs < 850) {
    return lastRxByte;
  }
  
  return -1;
}

int getRxSlot(int index) { return rxSlots[index]; }
int getTxSlot(int index) { return txSlots[index]; }
void setTxSlot(int index, int data) { txSlots[index] = data; }
int getSlotCount() { return PACKET_SLOT_COUNT; }
