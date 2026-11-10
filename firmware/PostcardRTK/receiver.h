// receiver.h - receiver task: LG290P UART, NMEA extraction, method ladder
#pragma once
#include "ladder.h"
#include <atomic>
#include <cstddef>

// SparkFun RTK Postcard: ESP32 <-> LG290P (as in SparkFun RTK Everywhere, Begin.ino)
static const int PIN_GNSS_RX = 21;    // ESP32 RX <- LG290P TX
static const int PIN_GNSS_TX = 22;    // ESP32 TX -> LG290P RX
static const int PIN_GNSS_RESET = 33; // high = run, low = reset

extern std::atomic<bool> configReady;
extern std::atomic<uint32_t> receiverBaud; // UART speed in use (found automatically if the setting is wrong)
extern std::atomic<bool> ladderSourceAvailable; // set by the main loop
extern std::atomic<bool> ladderAllowRtk;        // read by the main loop

void receiverInitShared(); // main loop, before configReady
void receiverSetup1();     // receiver task
void receiverLoop1();      // receiver task; returns quickly
void ladderConfigure();    // after a settings change
void getLadder(Ladder &out);
// The ladder's decision changed since the last call: copies the new reason and state
bool ladderTakeChange(char *reason, size_t n, LadderState &st);
