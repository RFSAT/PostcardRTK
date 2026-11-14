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
// The latest epoch, for status displays
struct EpochNow
{
    Method method = M_NONE;
    float estH = -1, estV = -1;
    CorrInfo corr;
    int sats = 0;
    uint32_t atMs = 0; // 0 = no epoch yet
};
void getEpochNow(EpochNow &out);
// The ladder's decision changed since the last call: copies the new reason and state
bool ladderTakeChange(char *reason, size_t n, LadderState &st);

#if BOARD_UBLOX
// F9R: a summary of where the u-blox receiver was found and how its configuration went (once per configuration, or when
// its version arrives); for the event log
bool ubloxTakeReport(char *out, size_t n);
const char *ubloxVersion(); // e.g. "HPS 1.30, protocol 33.30, ZED-F9R"; empty until known
#endif
