// receiver.h - receiver task: LG290P UART, NMEA extraction, method ladder
#pragma once
#include "ladder.h"
#include <atomic>
#include <cstddef>

#include "board.h" // pins of the receiver UART and reset line

extern std::atomic<bool> configReady;
extern std::atomic<uint32_t> receiverBaud;
extern std::atomic<uint8_t> currentMethod;    // Method of the last GGA (receiver task -> indicators)
extern std::atomic<uint32_t> currentMethodMs; // when it was set // UART speed in use (found automatically if the setting is wrong)
extern std::atomic<bool> ladderSourceAvailable; // set by the main loop
extern std::atomic<bool> ladderAllowRtk;        // read by the main loop

void receiverInitShared(); // main loop, before configReady
void receiverSetup1();     // receiver task
void receiverLoop1();      // receiver task; returns quickly
void ladderConfigure();    // after a settings change
void getLadder(Ladder &out);
// The ladder's decision changed since the last call: copies the new reason and state
bool ladderTakeChange(char *reason, size_t n, LadderState &st);
