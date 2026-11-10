// sky.h - satellites in view from GSV (position and C/N0 for every signal) and the ones used in the
// solution from GSA, for the Sky tab. Portable (no Arduino calls), unit-tested on the PC.
#pragma once
#include <cstddef>
#include <cstdint>

static const int SKY_MAX_SATS = 72;
static const int SKY_MAX_SIGS = 6;
static const uint32_t SKY_LIVE_MS = 5000;   // reported in GSV within this time = live
static const uint32_t SKY_KEEP_MS = 600000; // a satellite that lost lock stays (greyed) this long

struct SkySig
{
    uint8_t id = 0;   // NMEA signal ID (LG290P table 31), 0 = free slot
    uint8_t cn0 = 0;  // dB-Hz, 0 = not tracking
    uint32_t atMs = 0;
};

struct SkySat
{
    char sys = 0;     // G GPS, R GLONASS, E Galileo, C BeiDou, J QZSS, I NavIC, S SBAS; 0 = free slot
    uint16_t prn = 0; // satellite number in its system (SBAS: 120-158)
    int8_t el = -1;   // degrees, -1 = unknown
    int16_t az = -1;  // degrees, -1 = unknown
    uint32_t seenMs = 0, usedMs = 0;
    SkySig sig[SKY_MAX_SIGS];
};

// Band name for a system and NMEA signal ID ("L1", "E6", "B2b" ...), "?" when unknown
const char *skyBandName(char sys, int sigId);
// Order of a band from the highest frequency (L1/E1/B1) down, for sorting bars
int skyBandOrder(char sys, int sigId);

class SkyTable
{
  public:
    // Feed every GSV and GSA sentence (checksum already verified). Returns true if it was one of them.
    bool feed(const char *s, size_t len, uint32_t nowMs);
    // Drop satellites not reported for this long (signals: not reported for SKY_LIVE_MS)
    void expire(uint32_t nowMs, uint32_t maxAgeMs = SKY_KEEP_MS);
    static bool live(const SkySat &s, uint32_t nowMs) { return nowMs - s.seenMs < SKY_LIVE_MS; }
    uint32_t firstGsvMs() const { return _firstGsv; }
    int count() const { return SKY_MAX_SATS; }
    const SkySat &at(int i) const { return _s[i]; }
    // Used in the solution: listed in a GSA within the last 3 s
    static bool used(const SkySat &s, uint32_t nowMs) { return s.usedMs && nowMs - s.usedMs < 3000; }
    uint32_t lastGsvMs() const { return _lastGsv; }
    uint32_t lastGsaMs() const { return _lastGsa; }

  private:
    SkySat *find(char sys, uint16_t prn, bool create, uint32_t nowMs);
    SkySat _s[SKY_MAX_SATS];
    uint32_t _lastGsv = 0, _lastGsa = 0, _firstGsv = 0;
};

// Last known sky, saved to flash so the next start can show it (greyed) before the receiver locks
struct SkySnapSat
{
    char sys;
    uint8_t el, used, pad;
    uint16_t prn;
    int16_t az;
};
struct SkySnap
{
    uint32_t magic = 0; // SKY_SNAP_MAGIC when valid
    uint32_t n = 0;
    int64_t utc = -1;   // when it was taken (s since 1970)
    SkySnapSat s[SKY_MAX_SATS];
};
static const uint32_t SKY_SNAP_MAGIC = 0x534B5931; // "SKY1"
// Live satellites with a known position into a snapshot taken at utc. Returns the count.
int skySnapshot(const SkyTable &t, uint32_t nowMs, int64_t utc, SkySnap &out);
