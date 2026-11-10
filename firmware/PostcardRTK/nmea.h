// nmea.h - NMEA 0183 sentence assembly, checksum and GGA parsing (portable, unit-tested on the PC)
#pragma once
#include <cstddef>
#include <cstdint>

static const size_t NMEA_MAX = 255; // NMEA limit is 82; Quectel PQTMPPPNAV / PQTMNAV / PQTMPVT reach about 230

// Collects bytes from the receiver and returns complete sentences ("$...*hh\r\n").
// Bytes that are not part of an NMEA sentence (binary, RTCM echoes) are ignored here;
// the raw stream is still forwarded unchanged to the outputs.
class NmeaAssembler
{
  public:
    // Feed one byte. Returns true when a sentence with a valid checksum is ready in sentence().
    bool feed(uint8_t c);
    const char *sentence() const { return _out; }
    size_t length() const { return _outLen; }
    uint32_t good() const { return _good; }
    uint32_t bad() const { return _bad; }

  private:
    char _buf[NMEA_MAX + 3];
    size_t _len = 0;
    bool _in = false;
    char _out[NMEA_MAX + 3];
    size_t _outLen = 0;
    uint32_t _good = 0, _bad = 0;
};

// Validate "$....*hh" (CR/LF optional)
bool nmeaChecksumOk(const char *s, size_t len);
// Compute the XOR checksum of the characters between '$' and '*' (or end)
uint8_t nmeaChecksum(const char *body, size_t len);
// Append "*hh\r\n" to a "$BODY" command if it has no checksum. out must hold len+6.
size_t nmeaFinish(const char *in, char *out, size_t outSize);

struct GgaInfo
{
    bool valid = false;  // parsed a GGA
    int quality = 0;     // 0 none, 1 GNSS, 2 DGNSS, 4 RTK fixed, 5 RTK float, 6 dead reckoning
    int sats = 0;
    float hdop = 0;
    double lat = 0, lon = 0; // degrees, signed
    float alt = 0;           // metres above mean sea level
    float age = -1;          // seconds since last correction, -1 when empty
    int station = -1;        // reference station id, -1 when empty
    char utc[11] = {0};      // hhmmss.ss
};

// Build a GGA for a fixed position (quality 1), e.g. a bench position sent to VRS/nearest casters
// when the receiver has no fix. secOfDay = UTC seconds since midnight. Returns the length.
size_t nmeaMakeGga(double lat, double lon, float alt, uint32_t secOfDay, char *out, size_t outSize);

// Sentence type used for output filtering: "GGA" for $GNGGA/$GPGGA..., the full proprietary
// name for $P... sentences (e.g. "PQTMEPE"). Returns false for malformed input.
bool nmeaSentenceId(const char *s, size_t len, char *id, size_t idSize);
static const size_t NMEA_ID_MAX = 20; // "PQTMGEOFENCESTATUS" + NUL

// True for a receiver reply to a command rather than a periodic message:
// "$PQTM<cmd>,OK...", "$PQTM<cmd>,ERROR,...", and the version / id / serial-number replies.
bool nmeaIsReply(const char *s, size_t len);

// Accuracy messages (checksum already verified). Missing fields return false.
// GST: standard deviation of latitude, longitude, altitude (m)
bool nmeaParseGst(const char *s, size_t len, float &latStd, float &lonStd, float &altStd);
// PQTMEPE (version 2): estimated error north, east, down, 2D, 3D (m)
bool nmeaParseEpe(const char *s, size_t len, float &north, float &east, float &down, float &h2d, float &v3d);
// PQTMPPPNAV (version 1): PPP / Galileo HAS solution status
struct PppNav
{
    int sol = -1;        // 0 none, 1 single, 2 SBAS, 5 DGNSS, 6 PPP converging, 7 PPP converged, 8 RTK float, 12 RTK fixed
    float latStd = -1, lonStd = -1, altStd = -1;
    int diffId = -1;     // 9002 = Galileo HAS, 9001 = BDS B2b
    float diffAge = -1;  // s
    int satUsed = -1;
};
bool nmeaParsePppNav(const char *s, size_t len, PppNav &out);

// True when the sentence is a GGA from any talker ($GPGGA, $GNGGA, ...)
bool nmeaIsGga(const char *s, size_t len);
// Parse a GGA sentence (checksum already verified)
bool nmeaParseGga(const char *s, size_t len, GgaInfo &out);
