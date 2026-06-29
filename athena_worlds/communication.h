#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include <Arduino.h>
#include "core.h"

// === PEER LINK CONSTANTS ===
#define PACKET_SLOT_COUNT 5

// === PEER LINK FUNCTIONS ===
void initPeerLink();
void flushPeerFrame();
void sendPeerByte(int data);
void pollPeerFrame();
int readPeerByte();
int getRxSlot(int index);
int getTxSlot(int index);
void setTxSlot(int index, int data);
int getSlotCount();

#endif // COMMUNICATION_H
