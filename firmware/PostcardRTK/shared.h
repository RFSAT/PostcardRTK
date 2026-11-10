// shared.h - data exchanged between the main loop (network, radio) and the receiver task (UART)
#pragma once
#include "nmea.h"
#include "ring.h"
#include <atomic>
#include "plat.h"

// core 0 -> core 1: validated RTCM frames for the receiver
extern SpscRing<16384> rtcmToRx;
// core 0 -> core 1: receiver commands (boot commands, web console)
extern SpscRing<1024> cmdToRx;
// core 1 -> core 0: valid NMEA sentences for Bluetooth and TCP clients
extern SpscRing<16384> nmeaToNet;

// Latest GGA from the receiver (core 1 writes, core 0 reads)
struct SharedGga
{
    char sentence[NMEA_MAX + 3];
    size_t len;
    GgaInfo info;
    uint32_t atMs;
};
extern SharedGga sharedGga;
extern mutex_t ggaMutex;

// Counters, written by one core only
struct Stats
{
    std::atomic<uint32_t> uartRxBytes{0};
    std::atomic<uint32_t> uartTxBytes{0};
    std::atomic<uint32_t> nmeaGood{0};
    std::atomic<uint32_t> nmeaBad{0};
    std::atomic<uint32_t> rtcmFramesToRx{0};
    std::atomic<uint32_t> lastRtcmToRxMs{0};
};
extern Stats stats;

// NMEA sentence types seen from the receiver (core 1 writes, core 0 reads under typesMutex)
static const int MAX_NMEA_TYPES = 24;
struct NmeaTypeStat
{
    char id[NMEA_ID_MAX];
    uint32_t count;
    bool sent; // passes the output filter
};
extern NmeaTypeStat nmeaTypes[MAX_NMEA_TYPES];
extern int nmeaTypeCount;
extern mutex_t typesMutex;
// Replies of the receiver to commands ($PQTM...,OK / ERROR, version, id), for the web page
static const int RX_REPLIES = 24;
extern char rxReplies[RX_REPLIES][NMEA_MAX + 1];
extern uint32_t rxReplySeq; // total replies since start-up; newest is at (rxReplySeq-1) % RX_REPLIES
extern std::atomic<bool> core1Halt; // safe shutdown: core 1 stops touching the UART

// Latest accuracy estimates from the receiver (core 1 writes, core 0 reads under ggaMutex)
struct SharedAccuracy
{
    float gstLat = -1, gstLon = -1, gstAlt = -1;  uint32_t gstAtMs = 0;   // GST standard deviations
    float epe2d = -1, epeDown = -1;                uint32_t epeAtMs = 0;   // PQTMEPE
    PppNav ppp;                                    uint32_t pppAtMs = 0;   // PQTMPPPNAV (Galileo HAS)
};
void getAccuracy(SharedAccuracy &out);

// Positioning methods seen over time, their corrections, reference point, movement (core 1 writes, core 0 reads)
#include "methods.h"
void getMethods(MethodTracker &out, RefPoint &ref);
// Satellites in view (GSV) and used (GSA), for the Sky tab (core 1 writes, core 0 copies)
#include "sky.h"
void getSky(SkyTable &out);
// Current UTC (s since 1970) from the latest GGA and RMC date, -1 when not known
int64_t currentUtc();

// core 1 -> core 0: one HistRec every 10 s for the flash history
extern SpscRing<8192> histToCore0;
void setRefPoint(const RefPoint &r); // also call configSave() after updating cfg

// Copy the latest GGA (thread-safe). Returns false when none has arrived yet.
bool getLatestGga(SharedGga &out);
