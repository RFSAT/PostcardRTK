// methods.h - which positioning method the receiver is using, where its corrections come from and
// how old they are, per-method accuracy figures, error against a reference point, speed / movement,
// a 30-minute trace and the history record format.
// Portable (no Arduino calls), unit-tested on the PC.
//
// The LG290P computes ONE solution at a time. "Methods" here are the methods seen over time:
// the one running now, and the last figures of those that stopped. Nothing is computed in parallel.
#pragma once
#include "nmea.h"
#include <cstddef>
#include <cstdint>

enum Method : uint8_t
{
    M_NONE = 0,   // no fix / no data
    M_GNSS,       // standalone, no corrections
    M_SBAS,       // EGNOS
    M_DGNSS,      // code corrections from RTCM
    M_RTK_FLOAT,
    M_RTK_FIXED,
    M_HAS_CONV,   // Galileo HAS converging
    M_HAS_DONE,   // Galileo HAS converged
    M_DR,         // dead reckoning
    M_COUNT
};
const char *methodName(Method m);

// Groups shown as rows on the page
enum MethodGroup : uint8_t
{
    G_RTK = 0, // RTK fixed / float and DGNSS from RTCM
    G_HAS,
    G_SBAS,
    G_GNSS,
    G_COUNT
};
const char *groupName(MethodGroup g);
int methodGroup(Method m); // -1 for none / dead reckoning

// ppp may be nullptr when no fresh PQTMPPPNAV is available.
// rtcmActive: RTCM corrections were passed to the receiver in the last few seconds.
Method classifyMethod(const GgaInfo &g, const PppNav *ppp, bool rtcmActive);

struct RefPoint
{
    bool set = false;
    double lat = 0, lon = 0; // degrees
    float alt = 0;           // m above mean sea level (as in GGA)
};
// Horizontal and vertical distance of a position from the reference (m)
void refError(const RefPoint &r, double lat, double lon, float alt, float &errH, float &errV);

// Correction state of one epoch, as reported to the user
struct CorrInfo
{
    bool live = false; // corrections are still arriving (RTCM in the last few seconds; HAS/SBAS from the sky)
    float age = -1;    // age of the corrections in use (s), -1 = none / unknown
    int station = -1;  // RTCM base id, 9002 = Galileo HAS, 120-158 = EGNOS satellite (PRN), -1 = none
};
// Fill CorrInfo for a method. rtcmActive = RTCM passed to the receiver in the last few seconds.
CorrInfo correctionsFor(Method m, const GgaInfo &g, const PppNav *ppp, bool rtcmActive);

// Speed from RMC (field 7, knots) in m/s; -1 when missing
float rmcSpeed(const char *s, size_t len);

struct MethodStat
{
    bool seen = false;
    uint32_t startMs = 0, lastMs = 0; // start of the latest run, last epoch
    Method last = M_NONE;             // last method in this group
    float estH = -1, estV = -1;       // receiver's estimate at the last epoch (-1 = none)
    float errH = -1, errV = -1;       // error against the reference at the last epoch (-1 = no reference)
    float bestEstH = -1;              // best estimate during the latest run
    CorrInfo corr;                    // corrections at the last epoch
};

struct TraceSample
{
    uint32_t tSec;   // uptime (s)
    uint8_t method;  // Method
    float estH, errH; // -1 = none
};
static const int TRACE_N = 180;     // 30 min at one sample every 10 s
static const uint32_t TRACE_STEP_MS = 10000;

class MethodTracker
{
  public:
    // One epoch (per GGA). estH/estV = receiver estimate (-1 none); speed in m/s (-1 unknown).
    // Returns true when this epoch was added to the trace (one every 10 s): also the moment to log it.
    bool update(uint32_t nowMs, const GgaInfo &g, Method m, float estH, float estV, const RefPoint &ref, const CorrInfo &c,
                float speed);

    Method current() const { return _cur; }
    uint32_t currentSinceMs() const { return _curSince; }
    const MethodStat &stat(int grp) const { return _st[grp]; }

    // Movement, with hysteresis: moving above 0.5 m/s for 3 s, stationary below 0.3 m/s for 10 s
    float speed() const { return _speed; }
    bool moving() const { return _moving; }

    int traceCount() const { return _traceN; }
    const TraceSample &trace(int i) const; // 0 = oldest

  private:
    Method _cur = M_NONE;
    uint32_t _curSince = 0;
    MethodStat _st[G_COUNT];
    float _speed = -1;
    bool _moving = false;
    uint32_t _fastSince = 0, _slowSince = 0;
    TraceSample _tr[TRACE_N];
    int _traceN = 0, _traceHead = 0;
    uint32_t _lastTraceMs = 0;
    bool _traceStarted = false;
};

// One history record (kept in flash across restarts), written every 10 s. Format 2 (72 bytes).
struct HistRec
{
    double lat, lon;
    uint32_t ymd;   // UTC date as yyyymmdd from RMC, 0 = not known yet; 0xFFFFFFFF = restart marker
    uint32_t utcMs; // UTC time of day (ms) from GGA
    uint32_t upSec; // uptime
    float estH, estV, errH, errV; // -1 = none
    float alt, hdop;
    float speed;   // m/s, -1 = unknown
    float corrAge; // s, -1 = none
    int16_t station;
    uint8_t method, sats;
    uint8_t flags; // HF_LIVE, HF_MOVING
};
static const uint8_t HF_LIVE = 1, HF_MOVING = 2;
static const uint32_t HIST_RESTART = 0xFFFFFFFFu;
static const char HIST_CSV_HEADER[] =
    "date_utc,time_utc,uptime_s,method,corrections,correction_age_s,base_or_prn,est_h_m,est_v_m,err_h_m,err_v_m,"
    "lat_deg,lon_deg,alt_msl_m,speed_mps,moving,sats,hdop\r\n";
// UTC time of a record as seconds since 1970, -1 for a restart marker or a record without a date
int64_t histEpoch(const HistRec &r);

// One CSV line for a record. Returns its length.
size_t histCsvLine(const HistRec &r, char *out, size_t outSize);
// RMC date field ddmmyy -> yyyymmdd (0 when missing)
uint32_t rmcDate(const char *s, size_t len);
// GGA time hhmmss.ss -> ms of day (0 when missing)
uint32_t ggaTimeMs(const char *utc);
