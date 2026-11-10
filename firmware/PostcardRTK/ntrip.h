// ntrip.h - core 0: NTRIP client with ordered sources and failover
#pragma once
#include "rtcm.h"
#include "selector.h"
#include <Arduino.h>

enum class NtripState : uint8_t
{
    Off,        // no usable source configured
    NoNetwork,  // waiting for Wi-Fi
    Waiting,    // delay before the next attempt
    WaitFix,    // VRS/nearest source: waiting for a position to send
    Connecting, // TCP connect + request sent
    Reply,      // waiting for the caster's answer
    Streaming,  // receiving RTCM
    Stopped,    // every source rejected us; fix the settings
    Paused,     // the method ladder holds RTK back for a while
};

struct NtripStatus
{
    NtripState state;
    int source;            // active source index
    uint32_t bytes;        // bytes received from the caster since boot
    uint32_t frames;       // valid RTCM frames since boot
    uint32_t crcErrors;
    uint32_t switches;     // failovers since boot
    uint32_t streamSec;    // seconds on the current stream
    uint32_t lastFrameAgoMs;
    int stationId;         // from 1005/1006
    int backoffLevel;
    char lastEvent[96];    // human-readable last event
    uint32_t streamBytes;  // bytes received on the current stream
    uint32_t ggaSent;      // GGA sentences sent to casters since boot
    char lastGga[100];     // last GGA sent (without CR/LF)
};

void ntripSetup();                 // after configLoad()
void ntripStop();                  // safe shutdown: close the caster connection
void ntripRestart();               // after a correction-mode change
void ntripLoop(bool networkUp, bool allowed); // call often from the main loop; allowed = method ladder lets RTK run
void ntripStatus(NtripStatus &s);
const char *ntripStateName(NtripState s);
bool ntripStreaming();
void ntripEventLog(String &out);   // recent events, newest last
