#include <cmath>
// Host unit tests for the portable PostcardRTK modules.
// Build: g++ -std=c++17 -Wall -I../firmware/PostcardRTK test_main.cpp ../firmware/PostcardRTK/{nmea,rtcm,selector,ntrip_proto,methods}.cpp -o t && ./t
#include "methods.h"
#include "sky.h"
#include "evtrim.h"
#include "nmea.h"
#include "ntrip_proto.h"
#include "ring.h"
#include "rtcm.h"
#include "selector.h"
#include "ladder.h"
#include "ublox.h"
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

static int fails = 0, checks = 0;
#define CHECK(c)                                                                                                       \
    do                                                                                                                 \
    {                                                                                                                  \
        checks++;                                                                                                      \
        if (!(c))                                                                                                      \
        {                                                                                                              \
            printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #c);                                                       \
            fails++;                                                                                                   \
        }                                                                                                              \
    } while (0)

static std::string withChecksum(const char *body)
{
    char out[200];
    nmeaFinish(body, out, sizeof(out));
    return out;
}

static void testNmea()
{
    // Known sentence from the NMEA reference (checksum 47)
    const char *ref = "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47\r\n";
    CHECK(nmeaChecksumOk(ref, strlen(ref)));
    std::string bad = ref;
    bad[10] = '9';
    CHECK(!nmeaChecksumOk(bad.c_str(), bad.size()));

    // Assembler across noise, binary and a damaged sentence
    NmeaAssembler a;
    std::string gga = withChecksum("$GNGGA,101530.00,3758.1234567,N,02343.7654321,E,4,32,0.5,120.123,M,35.1,M,1.0,0042");
    std::string stream = std::string("\xD3\x00\x13garbage") + gga + "$GNRMC,broken*00\r\n" + gga;
    int got = 0;
    for (unsigned char c : stream)
        if (a.feed(c))
        {
            got++;
            CHECK(std::string(a.sentence()) == gga);
        }
    CHECK(got == 2);
    CHECK(a.bad() >= 1);

    GgaInfo g;
    CHECK(nmeaIsGga(gga.c_str(), gga.size()));
    CHECK(nmeaParseGga(gga.c_str(), gga.size(), g));
    CHECK(g.quality == 4);
    CHECK(g.sats == 32);
    CHECK(g.age > 0.99f && g.age < 1.01f);
    CHECK(g.station == 42);
    CHECK(g.lat > 37.96872 && g.lat < 37.96873);
    CHECK(g.lon > 23.72942 && g.lon < 23.72943);
    CHECK(strcmp(g.utc, "101530.00") == 0);

    std::string noFix = withChecksum("$GNGGA,101531.00,,,,,0,00,99.99,,,,,,");
    CHECK(nmeaParseGga(noFix.c_str(), noFix.size(), g));
    CHECK(g.quality == 0 && g.age < 0 && g.station == -1);

    // Bench GGA: Athens, valid checksum, parses back
    char bench[128];
    size_t bl = nmeaMakeGga(37.968723, 23.729420, 120.5f, 10 * 3600 + 15 * 60 + 30, bench, sizeof(bench));
    CHECK(bl > 40 && nmeaChecksumOk(bench, bl));
    GgaInfo b;
    CHECK(nmeaParseGga(bench, bl, b) && b.quality == 1);
    CHECK(b.lat > 37.96872 && b.lat < 37.96873 && b.lon > 23.72941 && b.lon < 23.72943);
    CHECK(strncmp(b.utc, "101530", 6) == 0);
    bl = nmeaMakeGga(-33.5, -70.25, 0, 0, bench, sizeof(bench));
    CHECK(nmeaParseGga(bench, bl, b) && b.lat < -33.49 && b.lon < -70.24);

    char id[16];
    CHECK(nmeaSentenceId(gga.c_str(), gga.size(), id, sizeof(id)) && strcmp(id, "GGA") == 0);
    const char *gsv = "$GLGSV,3,1,10,65,12,045,30*6A";
    CHECK(nmeaSentenceId(gsv, strlen(gsv), id, sizeof(id)) && strcmp(id, "GSV") == 0);
    const char *pq = "$PQTMEPE,2,0.012,0.010,0.025,0.016,0.030*6E";
    CHECK(nmeaSentenceId(pq, strlen(pq), id, sizeof(id)) && strcmp(id, "PQTMEPE") == 0);

    // Command replies vs periodic messages; long proprietary ids fit
    const char *r1 = "$PQTMCFGMSGRATE,OK*29";
    const char *r2 = "$PQTMCFGMSGRATE,ERROR,1*xx";
    const char *r3 = "$PQTMVERNO,LG290P03AANR01A06S,2024/11/27,17:28:51*3F";
    const char *r4 = "$PQTMCFGODO,OK,1,10.5*1D";
    CHECK(nmeaIsReply(r1, strlen(r1)) && nmeaIsReply(r2, strlen(r2)) && nmeaIsReply(r3, strlen(r3)) && nmeaIsReply(r4, strlen(r4)));
    CHECK(!nmeaIsReply(pq, strlen(pq)) && !nmeaIsReply(gsv, strlen(gsv)));
    const char *okv = "$PQTMODO,1,120635.000,1,112.3*6E";
    CHECK(!nmeaIsReply(okv, strlen(okv)));
    const char *gf = "$PQTMGEOFENCESTATUS,1,120635.000,0,0,0,0*00";
    char id20[NMEA_ID_MAX];
    CHECK(nmeaSentenceId(gf, strlen(gf), id20, sizeof(id20)) && strcmp(id20, "PQTMGEOFENCESTATUS") == 0);

    // Accuracy messages
    {
        float a, b, c;
        const char *gst = "$GNGST,114915.60,1.2,0.9,0.7,45.0,0.012,0.015,0.031*5A";
        CHECK(nmeaParseGst(gst, strlen(gst), a, b, c) && fabsf(a - 0.012f) < 1e-6 && fabsf(b - 0.015f) < 1e-6 && fabsf(c - 0.031f) < 1e-6);
        const char *gst0 = "$GNGST,114915.60,,,,,,,*5A";
        CHECK(!nmeaParseGst(gst0, strlen(gst0), a, b, c));
        float n, e, d, h, v;
        const char *epe = "$PQTMEPE,2,0.021,0.018,0.040,0.028,0.049*00";
        CHECK(nmeaParseEpe(epe, strlen(epe), n, e, d, h, v) && fabsf(h - 0.028f) < 1e-6 && fabsf(d - 0.040f) < 1e-6);
        // PQTMPPPNAV: sol type field 10, stds 18-20, diff id 23, age 24, sats used 27
        const char *ppp = "$PQTMPPPNAV,1,1,1,190423.000,20241224,212681000,2346,18,1,,7,,31.45874521,117.41532415,45.123,0.5,,,0.081,0.064,0.150,,,9002,12.0,,31,24,,,,,,,0.01,0.02,0.01,0.02,,,123.4,,*00";
        PppNav pn;
        CHECK(nmeaParsePppNav(ppp, strlen(ppp), pn) && pn.sol == 7 && pn.diffId == 9002 && fabsf(pn.diffAge - 12.0f) < 1e-6 &&
              fabsf(pn.latStd - 0.081f) < 1e-6 && fabsf(pn.altStd - 0.150f) < 1e-6 && pn.satUsed == 24);
    }

    // nmeaFinish keeps an existing checksum
    char out[64];
    nmeaFinish("$PQTMVERNO*58", out, sizeof(out));
    CHECK(strcmp(out, "$PQTMVERNO*58\r\n") == 0);
}

static std::vector<uint8_t> makeRtcm(uint16_t type, uint16_t station, size_t extra)
{
    std::vector<uint8_t> payload(2 + 1 + extra, 0);
    payload[0] = (uint8_t)(type >> 4);
    payload[1] = (uint8_t)((type & 0x0F) << 4) | (uint8_t)((station >> 8) & 0x0F);
    payload[2] = (uint8_t)(station & 0xFF);
    for (size_t i = 0; i < extra; i++)
        payload[3 + i] = (uint8_t)(i * 7);
    std::vector<uint8_t> f = {0xD3, (uint8_t)(payload.size() >> 8), (uint8_t)(payload.size() & 0xFF)};
    f.insert(f.end(), payload.begin(), payload.end());
    uint32_t crc = rtcmCrc24q(f.data(), f.size());
    f.push_back((uint8_t)(crc >> 16));
    f.push_back((uint8_t)(crc >> 8));
    f.push_back((uint8_t)crc);
    return f;
}

static void testRtcm()
{
    const char *v = "123456789";
    CHECK(rtcmCrc24q((const uint8_t *)v, 9) == 0xCDE703);

    RtcmFramer r;
    auto f1005 = makeRtcm(1005, 2003, 16);
    auto f1077 = makeRtcm(1077, 2003, 300);
    std::vector<uint8_t> s = {'I', 'C', 'Y', 0xD3, 0xFF}; // junk incl. a false preamble
    s.insert(s.end(), f1005.begin(), f1005.end());
    auto broken = f1077;
    broken[50] ^= 0x55;
    s.insert(s.end(), broken.begin(), broken.end());
    s.insert(s.end(), f1077.begin(), f1077.end());
    std::vector<uint16_t> types;
    for (uint8_t b : s)
        if (r.feed(b))
        {
            types.push_back(r.messageType());
            CHECK(r.frameLength() == (r.messageType() == 1005 ? f1005.size() : f1077.size()));
        }
    CHECK(types.size() == 2);
    CHECK(types.size() == 2 && types[0] == 1005 && types[1] == 1077);
    CHECK(r.stationId() == 2003);
    CHECK(r.crcErrors() == 1);
}

static void testSelector()
{
    SourceSelector s;
    bool v[4] = {true, true, false, true}; // own base, VRS, (empty), EUREF
    s.setSources(4, v);
    CHECK(s.active() == 0 && s.validCount() == 3);

    auto d = s.fail(false, 0);
    CHECK(d.switched && d.source == 1 && d.delayMs == 2000 && !d.backoff);
    d = s.fail(false, 0);
    CHECK(d.source == 3 && !d.backoff); // skips the empty slot
    d = s.fail(false, 0);
    CHECK(d.backoff && d.source == 0 && d.delayMs == 15000);
    // Second full cycle backs off longer
    s.fail(false, 0);
    s.fail(false, 0);
    d = s.fail(false, 0);
    CHECK(d.backoff && d.delayMs == 30000);

    // A good connection resets the back-off; fail-back after the interval
    d = s.fail(false, 0);
    CHECK(d.source == 1);
    s.connected(1000);
    CHECK(!s.failbackDue(1000 + 599000));
    CHECK(s.failbackDue(1000 + 600000));
    s.reset(false);
    CHECK(s.active() == 0);

    // Hard failure disables the primary; no fail-back to it
    d = s.fail(true, 0);
    CHECK(d.source == 1 && s.disabled(0));
    s.connected(0);
    CHECK(!s.failbackDue(10000000));
    d = s.fail(true, 0);
    CHECK(d.source == 3);
    d = s.fail(false, 0);
    CHECK(d.backoff && d.source == 3); // only one left
    d = s.fail(true, 0);
    CHECK(d.stopped);
    s.reset(true);
    CHECK(s.usableCount() == 3 && s.active() == 0);

    // Failover off: only the first configured source is used
    s.setFailover(false);
    s.reset(true);
    CHECK(s.validCount() == 1);
    d = s.fail(false, 0);
    CHECK(d.backoff && d.source == 0);

    // Single source with the primary slot empty
    SourceSelector t;
    bool v2[3] = {false, true, false};
    t.setSources(3, v2);
    CHECK(t.active() == 1);
}

static void testNtrip()
{
    char b64[64];
    base64Encode((const uint8_t *)"user:pass", 9, b64, sizeof(b64));
    CHECK(strcmp(b64, "dXNlcjpwYXNz") == 0);

    char req[512];
    CHECK(ntripBuildRequest(req, sizeof(req), "gr.nrtk.eu", 7801, "MSM_VRS", "user", "pass", false, "PostcardRTK/0.1") > 0);
    CHECK(strstr(req, "GET /MSM_VRS HTTP/1.0\r\n") != nullptr);
    CHECK(strstr(req, "Authorization: Basic dXNlcjpwYXNz\r\n") != nullptr);
    CHECK(strstr(req, "\r\n\r\n") != nullptr);
    ntripBuildRequest(req, sizeof(req), "caster", 2101, "M", "", "", true, "X");
    CHECK(strstr(req, "Ntrip-Version: Ntrip/2.0") && !strstr(req, "Authorization"));

    NtripResponse r;
    size_t off;
    const char *icy = "ICY 200 OK\r\n\xD3\x00";
    r.reset();
    CHECK(r.feed((const uint8_t *)icy, 4, &off) == NtripReply::Incomplete);
    CHECK(r.feed((const uint8_t *)icy + 4, 10, &off) == NtripReply::Ok);
    CHECK(off == 8); // stream starts after "ICY 200 OK\r\n"

    const char *v2ok = "HTTP/1.1 200 OK\r\nContent-Type: gnss/data\r\nTransfer-Encoding: chunked\r\n\r\n5\r\nabcde\r\n";
    r.reset();
    CHECK(r.feed((const uint8_t *)v2ok, strlen(v2ok), &off) == NtripReply::Ok && r.chunked());
    ChunkDecoder cd;
    cd.reset();
    uint8_t out[64];
    size_t n = cd.decode((const uint8_t *)v2ok + off, strlen(v2ok) - off, out);
    CHECK(n == 5 && memcmp(out, "abcde", 5) == 0 && !cd.error());
    // Chunks split across reads
    const char *split[] = {"3", "\r\nxyz", "\r", "\n2\r\n1", "2\r\n"};
    size_t total = 0;
    for (auto p : split)
        total += cd.decode((const uint8_t *)p, strlen(p), out + total);
    CHECK(total == 5 && memcmp(out, "xyz12", 5) == 0);

    const char *st = "SOURCETABLE 200 OK\r\nServer: x\r\n";
    r.reset();
    CHECK(r.feed((const uint8_t *)st, strlen(st), &off) == NtripReply::Sourcetable);
    const char *st2 = "HTTP/1.1 200 OK\r\nContent-Type: gnss/sourcetable\r\n\r\nSTR;...";
    r.reset();
    CHECK(r.feed((const uint8_t *)st2, strlen(st2), &off) == NtripReply::Sourcetable);
    const char *u = "HTTP/1.0 401 Unauthorized\r\nWWW-Authenticate: Basic\r\n\r\n";
    r.reset();
    CHECK(r.feed((const uint8_t *)u, strlen(u), &off) == NtripReply::Unauthorized);
    const char *busy = "HTTP/1.1 503 Service Unavailable\r\n\r\n";
    r.reset();
    CHECK(r.feed((const uint8_t *)busy, strlen(busy), &off) == NtripReply::SoftError);
    const char *banned = "ERROR - Banned\r\n";
    r.reset();
    CHECK(r.feed((const uint8_t *)banned, strlen(banned), &off) == NtripReply::SoftError);
}

static void testRing()
{
    SpscRing<16> ring;
    uint8_t buf[32];
    CHECK(ring.write((const uint8_t *)"0123456789", 10));
    CHECK(!ring.write((const uint8_t *)"0123456789", 10)); // all or nothing
    CHECK(ring.dropped() == 1);
    CHECK(ring.read(buf, 4) == 4 && memcmp(buf, "0123", 4) == 0);
    CHECK(ring.write((const uint8_t *)"abcdefghij", 10)); // wraps around
    size_t n = ring.read(buf, sizeof(buf));
    CHECK(n == 16 && memcmp(buf, "456789abcdefghij", 16) == 0);
    CHECK(ring.available() == 0);
}

static GgaInfo mkGga(int q, int station, double lat = 37.9, double lon = 23.7, float alt = 280)
{
    GgaInfo g;
    g.valid = true; g.quality = q; g.station = station; g.lat = lat; g.lon = lon; g.alt = alt;
    return g;
}

static void testMethods()
{
    // Classification
    PppNav p; p.sol = 6; p.diffId = 9002;
    CHECK(classifyMethod(mkGga(2, -1), nullptr, false) == M_SBAS);
    CHECK(classifyMethod(mkGga(2, 124), nullptr, false) == M_SBAS);   // EGNOS PRN as station
    CHECK(classifyMethod(mkGga(2, 124), nullptr, true) == M_DGNSS);   // RTCM flowing: a real base
    CHECK(classifyMethod(mkGga(2, 27), nullptr, false) == M_DGNSS);
    CHECK(classifyMethod(mkGga(2, 0), nullptr, false) == M_SBAS);     // u-blox SBAS: station 0
    CHECK(classifyMethod(mkGga(2, 0), nullptr, true) == M_DGNSS);     // base id 0 with RTCM flowing
    CHECK(classifyMethod(mkGga(6, -1), nullptr, false) == M_DR);      // ZED-F9R dead reckoning only
    CHECK(classifyMethod(mkGga(4, 27), nullptr, true) == M_RTK_FIXED);
    CHECK(classifyMethod(mkGga(5, 27), nullptr, true) == M_RTK_FLOAT);
    CHECK(classifyMethod(mkGga(5, 9002), nullptr, false) == M_HAS_CONV);
    CHECK(classifyMethod(mkGga(2, -1), &p, false) == M_HAS_CONV);
    p.sol = 7;
    CHECK(classifyMethod(mkGga(5, 9002), &p, false) == M_HAS_DONE);
    p.sol = 2;
    CHECK(classifyMethod(mkGga(2, -1), &p, false) == M_SBAS);
    CHECK(classifyMethod(mkGga(1, -1), nullptr, false) == M_GNSS);
    CHECK(classifyMethod(mkGga(0, -1), nullptr, false) == M_NONE);

    // Reference error: 1e-5 deg latitude = 1.113 m north
    RefPoint r; r.set = true; r.lat = 37.9; r.lon = 23.7; r.alt = 280;
    float eh, ev;
    refError(r, 37.90001, 23.7, 281.5f, eh, ev);
    CHECK(eh > 1.10f && eh < 1.12f);
    CHECK(ev > 1.49f && ev < 1.51f);
    refError(r, 37.9, 23.70001, 280, eh, ev); // east, scaled by cos(lat)
    CHECK(eh > 0.87f && eh < 0.89f);
    RefPoint none;
    refError(none, 1, 1, 1, eh, ev);
    CHECK(eh < 0 && ev < 0);

    // Corrections per method
    GgaInfo gr = mkGga(4, 27); gr.age = 1.2f;
    CorrInfo c = correctionsFor(M_RTK_FIXED, gr, nullptr, true);
    CHECK(c.live && c.station == 27 && c.age > 1.1f);
    gr.age = 85;
    c = correctionsFor(M_RTK_FIXED, gr, nullptr, false);
    CHECK(!c.live && c.age > 84.9f);
    PppNav ph; ph.sol = 6; ph.diffId = 9002; ph.diffAge = 4;
    c = correctionsFor(M_HAS_CONV, mkGga(5, 9002), &ph, false);
    CHECK(c.live && c.station == 9002 && c.age > 3.9f && c.age < 4.1f);
    c = correctionsFor(M_SBAS, mkGga(2, 124), nullptr, false);
    CHECK(c.live && c.station == 124);
    c = correctionsFor(M_SBAS, mkGga(2, -1), nullptr, false);
    CHECK(c.station == -1);
    c = correctionsFor(M_GNSS, mkGga(1, -1), nullptr, false);
    CHECK(!c.live && c.age < 0 && c.station == -1);

    // Speed from RMC (knots -> m/s)
    std::string rmcS = withChecksum("$GNRMC,120000.00,A,3758.1234567,N,02343.5678901,E,19.438,90.0,061126,,,R,V");
    float v = rmcSpeed(rmcS.c_str(), rmcS.size());
    CHECK(v > 9.99f && v < 10.01f);
    std::string rmcN = withChecksum("$GNRMC,120000.00,V,,,,,,,061126,,,N,V");
    CHECK(rmcSpeed(rmcN.c_str(), rmcN.size()) < 0);

    // Tracker: stopped methods keep their last figures, corrections included
    MethodTracker t;
    uint32_t now = 1000;
    CorrInfo cs; cs.live = true; cs.station = 124;
    t.update(now, mkGga(2, -1, 37.9000010, 23.7), M_SBAS, 0.67f, 1.0f, r, cs, 0);
    CHECK(t.current() == M_SBAS && t.stat(G_SBAS).seen && t.stat(G_SBAS).estH > 0.66f);
    CHECK(t.stat(G_SBAS).errH > 0.1f && t.stat(G_SBAS).errH < 0.12f && t.stat(G_SBAS).corr.station == 124);
    now += 1000;
    CorrInfo ch; ch.live = true; ch.station = 9002; ch.age = 3;
    t.update(now, mkGga(5, 9002, 37.90002, 23.7), M_HAS_CONV, 1.9f, 3.0f, r, ch, 0);
    CHECK(t.current() == M_HAS_CONV && t.stat(G_SBAS).lastMs == 1000 && t.stat(G_HAS).startMs == 2000);
    CHECK(t.stat(G_HAS).corr.age > 2.9f);
    CHECK(t.traceCount() == 1);
    t.update(now + 11000, mkGga(5, 9002), M_HAS_CONV, 0.5f, 1.0f, r, ch, 0);
    CHECK(t.traceCount() == 2 && t.trace(1).estH < 0.51f && t.trace(0).method == M_SBAS);

    // Movement hysteresis: moving after 3 s above 0.5 m/s, stationary after 10 s below 0.3 m/s
    MethodTracker mv;
    CorrInfo cn;
    uint32_t tm = 1000;
    for (int k = 0; k < 3; k++, tm += 1000) mv.update(tm, mkGga(1, -1), M_GNSS, 2, 3, r, cn, 10);
    CHECK(!mv.moving());
    mv.update(tm, mkGga(1, -1), M_GNSS, 2, 3, r, cn, 10); tm += 1000;
    CHECK(mv.moving());
    for (int k = 0; k < 5; k++, tm += 1000) mv.update(tm, mkGga(1, -1), M_GNSS, 2, 3, r, cn, 0.1f); // traffic light
    CHECK(mv.moving());
    for (int k = 0; k < 6; k++, tm += 1000) mv.update(tm, mkGga(1, -1), M_GNSS, 2, 3, r, cn, 0.1f);
    CHECK(!mv.moving());
    mv.update(tm, mkGga(1, -1), M_GNSS, 2, 3, r, cn, -1);
    CHECK(mv.speed() < 0 && !mv.moving());
}

static void testHistory()
{
    std::string rmc = withChecksum("$GNRMC,120000.00,A,3758.1234567,N,02343.5678901,E,0.01,0.0,051126,,,R,V");
    CHECK(rmcDate(rmc.c_str(), rmc.size()) == 20261105);
    std::string rmc0 = withChecksum("$GNRMC,,V,,,,,,,,,,N,V");
    CHECK(rmcDate(rmc0.c_str(), rmc0.size()) == 0);
    CHECK(ggaTimeMs("120000.50") == 43200500);
    CHECK(ggaTimeMs("") == 0);
    HistRec r{};
    r.ymd = 20261105; r.utcMs = 43200500; r.upSec = 1234; r.estH = 0.67f; r.estV = 1.0f; r.errH = -1; r.errV = -1;
    r.lat = 37.9; r.lon = -23.7; r.alt = 280.5f; r.hdop = 0.6f; r.method = M_SBAS; r.sats = 30;
    r.speed = 12.5f; r.corrAge = -1; r.station = 124; r.flags = HF_LIVE | HF_MOVING;
    char line[240];
    histCsvLine(r, line, sizeof(line));
    CHECK(std::string(line) == "2026-11-05,12:00:00.5,1234,SBAS (EGNOS),live,,124,0.670,1.000,,,37.900000000,-23.700000000,280.500,12.50,yes,30,0.60\r\n");
    HistRec k{};
    k.ymd = 20261106; k.utcMs = 0; k.method = M_RTK_FIXED; k.corrAge = 85; k.station = 27; k.flags = 0; k.speed = -1;
    k.estH = k.estV = k.errH = k.errV = -1;
    histCsvLine(k, line, sizeof(line));
    CHECK(strstr(line, ",RTK fixed,stale,85.0,27,") != nullptr && strstr(line, ",,,0,") != nullptr);
    HistRec b{}; b.ymd = HIST_RESTART; b.upSec = 2;
    histCsvLine(b, line, sizeof(line));
    CHECK(std::string(line) == ",,2,restart,,,,,,,,,,,,,,\r\n");
    // header and lines have the same number of columns
    int ch = 0, cl = 0, cb = 0;
    for (const char *p = HIST_CSV_HEADER; *p; p++) ch += *p == ',';
    histCsvLine(r, line, sizeof(line));
    for (const char *p = line; *p; p++) cl += *p == ',';
    histCsvLine(b, line, sizeof(line));
    for (const char *p = line; *p; p++) cb += *p == ',';
    CHECK(ch == cl && ch == cb && ch == 17);
    CHECK(sizeof(HistRec) == 72);
    HistRec e{}; e.ymd = 20261106; e.utcMs = 43200500;
    CHECK(histEpoch(e) == 1793966400LL); // 2026-11-06 12:00:00 UTC (calendar.timegm)
    e.ymd = 19700101; e.utcMs = 0;
    CHECK(histEpoch(e) == 0);
    e.ymd = 20240229; e.utcMs = 86399000;
    CHECK(histEpoch(e) == 1709251199LL);
    e.ymd = 0;
    CHECK(histEpoch(e) == -1);
    e.ymd = HIST_RESTART;
    CHECK(histEpoch(e) == -1);
}

static void testSky()
{
    SkyTable t;
    uint32_t now = 1000;
    auto feed = [&](const char *body) { std::string x = withChecksum(body); return t.feed(x.c_str(), x.size(), now); };
    // the spec's examples: GPS L1, L2C, L5 for the same satellites; GLONASS G1 and G2
    CHECK(feed("$GPGSV,2,1,05,10,77,300,36,12,40,082,31,23,58,153,35,25,46,137,33,1"));
    feed("$GPGSV,2,2,05,32,45,316,34,1");
    feed("$GPGSV,2,1,05,10,77,300,31,12,40,082,25,23,58,153,29,25,46,137,28,6");
    feed("$GPGSV,1,1,04,10,77,300,32,23,58,153,30,25,46,137,30,32,45,316,26,8");
    feed("$GLGSV,1,1,03,67,57,036,37,68,30,328,34,78,53,184,27,1");
    feed("$GAGSV,1,1,02,05,40,120,41,12,20,250,,5"); // E6: one tracked, one not
    feed("$GBGSV,1,1,01,23,60,100,44,B");               // BDS B2I (hex signal ID)
    feed("$GPGSV,1,1,01,36,45,180,40,1");               // GPS-talker ID 36 = SBAS PRN 123
    CHECK(!feed("$GNGGA,120000.00,,,,,0,00,,,M,,M,,"));
    const SkySat *g10 = nullptr, *r3 = nullptr, *e5 = nullptr, *e12 = nullptr, *c23 = nullptr, *s123 = nullptr;
    int nSats = 0;
    for (int i = 0; i < t.count(); i++)
    {
        const SkySat &x = t.at(i);
        if (!x.sys) continue;
        nSats++;
        if (x.sys == 'G' && x.prn == 10) g10 = &x;
        if (x.sys == 'R' && x.prn == 3) r3 = &x;
        if (x.sys == 'E' && x.prn == 5) e5 = &x;
        if (x.sys == 'E' && x.prn == 12) e12 = &x;
        if (x.sys == 'C' && x.prn == 23) c23 = &x;
        if (x.sys == 'S' && x.prn == 123) s123 = &x;
    }
    CHECK(nSats == 5 + 3 + 2 + 1 + 1);
    CHECK(g10 && g10->el == 77 && g10->az == 300);
    int l1 = 0, l2 = 0, l5 = 0;
    for (auto &g : g10->sig) { if (g.id == 1) l1 = g.cn0; if (g.id == 6) l2 = g.cn0; if (g.id == 8) l5 = g.cn0; }
    CHECK(l1 == 36 && l2 == 31 && l5 == 32);
    CHECK(r3 && r3->el == 57);                 // GLONASS 67 -> slot 3
    CHECK(e5 && e5->sig[0].id == 5 && e5->sig[0].cn0 == 41);
    CHECK(e12 && e12->sig[0].cn0 == 0);        // in view, E6 not tracked
    CHECK(c23 && c23->sig[0].id == 11);
    CHECK(s123 && s123->el == 45);
    CHECK(!strcmp(skyBandName('E', 5), "E6") && !strcmp(skyBandName('C', 11), "B2I") && !strcmp(skyBandName('R', 3), "G2"));
    CHECK(skyBandOrder('E', 7) < skyBandOrder('E', 5) && skyBandOrder('E', 5) < skyBandOrder('E', 1));
    // used in the solution (GSA with system ID)
    CHECK(feed("$GNGSA,A,3,10,23,,,,,,,,,,,1.2,0.7,1.0,1"));
    feed("$GNGSA,A,3,05,,,,,,,,,,,,1.2,0.7,1.0,3");
    CHECK(SkyTable::used(*g10, now) && SkyTable::used(*e5, now) && !SkyTable::used(*e12, now));
    CHECK(!SkyTable::used(*g10, now + 4000));
    // lost lock: kept (not live, no signal level) for up to 10 min, then dropped
    now += 6000;
    feed("$GAGSV,1,1,01,05,41,121,40,7");
    t.expire(now);
    int nLive = 0, nKept = 0;
    for (int i = 0; i < t.count(); i++) if (t.at(i).sys) { nKept++; if (SkyTable::live(t.at(i), now)) nLive++; }
    CHECK(nLive == 1 && nKept == 12);
    CHECK(g10->sig[0].id == 0 && g10->el == 77); // signal gone, position kept
    CHECK(t.firstGsvMs() == 1000);
    SkySnap snap;
    CHECK(skySnapshot(t, now, 1793966400LL, snap) == 1 && snap.s[0].sys == 'E' && snap.s[0].prn == 5 && snap.s[0].el == 41);
    CHECK(snap.magic == SKY_SNAP_MAGIC && snap.utc == 1793966400LL);
    now += SKY_KEEP_MS;
    t.expire(now);
    nKept = 0;
    for (int i = 0; i < t.count(); i++) if (t.at(i).sys) nKept++;
    CHECK(nKept == 1);
    now += 1;
    t.expire(now);
    nKept = 0;
    for (int i = 0; i < t.count(); i++) if (t.at(i).sys) nKept++;
    CHECK(nKept == 0);
}

static void testEvTrim()
{
    std::string a = "S\t1\t-\nE\t1\t10\t-\told one\nE\t1\t20\t-\told two\n";
    std::string b = "S\t2\t-\nE\t2\t5\t-\tnew one\nE\t2\t6\t-\tnew two\n";
    CHECK(evTrim(a + b, 2, 1000) == a + b);                      // fits: unchanged
    std::string t1 = evTrim(a + b, 2, b.size() + 10);
    CHECK(t1 == b);                                              // earlier session dropped first
    std::string t2 = evTrim(a + b, 2, (a + b).size() - 18);
    CHECK(t2.find("old one") == std::string::npos && t2.find("old two") != std::string::npos && t2.find("new one") != std::string::npos);
    // current session alone too big: its oldest events go, with a marker after its start line
    std::string big = "S\t3\t-\n";
    for (int i = 0; i < 20; i++) big += "E\t3\t" + std::to_string(i) + "\t-\tevent number " + std::to_string(i) + "\n";
    std::string t3 = evTrim(a + big, 3, 200);
    CHECK(t3.size() <= 200);
    CHECK(t3.compare(0, 6, "S\t3\t-\n") == 0 && t3.find("D\t3\t") == 6);
    CHECK(t3.find("event number 19") != std::string::npos && t3.find("event number 0\n") == std::string::npos);
    // a second trim adds to the dropped count instead of a second marker
    std::string t4 = evTrim(t3 + "E\t3\t97\t-\tone more event here\nE\t3\t98\t-\tand another event\n", 3, 200);
    CHECK(t4.size() <= 200 && t4.find("and another event") != std::string::npos);
    size_t d1 = t4.find("D\t3\t"), d2 = t4.find("D\t3\t", d1 + 1);
    CHECK(d1 != std::string::npos && d2 == std::string::npos);
    int n3 = atoi(t3.c_str() + t3.find("D\t3\t") + 4), n4 = atoi(t4.c_str() + d1 + 4);
    CHECK(n4 > n3);
    CHECK(evSession("E\t42\tx", 6) == 42 && evSession("x", 1) == 0);
}

// Feed the ladder one epoch per second for `sec` seconds
static void run(Ladder &l, uint32_t &t, int sec, bool src, bool live, Method m, float est)
{
    for (int i = 0; i < sec; i++)
    {
        LadderInput in;
        in.nowMs = t;
        in.sourceAvailable = src;
        in.correctionsLive = live;
        in.method = m;
        in.estH = est;
        l.update(in);
        t += 1000;
    }
}

static void testLadder()
{
    // 1. No source: RTK allowed (as soon as one appears), receiver on its own
    {
        Ladder l;
        uint32_t t = 1000;
        run(l, t, 5, false, false, M_SBAS, 0.9f);
        CHECK(l.state() == LS_NO_SOURCE && l.allowRtk());
        CHECK(l.altBest() > 0.89f && l.altBest() < 0.91f);
        CHECK(l.takeChange() && !l.takeChange());
    }
    // 2. RTK fixed: stays RTK, never paused
    {
        Ladder l;
        uint32_t t = 1000;
        run(l, t, 30, true, true, M_RTK_FLOAT, 2.0f);
        CHECK(l.state() == LS_RTK && l.allowRtk());
        run(l, t, 3600, true, true, M_RTK_FIXED, 0.02f);
        CHECK(l.state() == LS_RTK && l.allowRtk() && l.pauseCount() == 0);
        CHECK(strcmp(l.reason(), "RTK fixed") == 0);
    }
    // 3. The balcony case: RTK float stuck at 4 m, SBAS seen at 0.8 m before -> paused after the trial
    {
        Ladder l;
        uint32_t t = 1000;
        run(l, t, 60, false, false, M_SBAS, 0.8f);
        run(l, t, 299, true, true, M_RTK_FLOAT, 4.0f);
        CHECK(l.state() == LS_RTK && l.allowRtk()); // still in the 5 min trial
        run(l, t, 2, true, true, M_RTK_FLOAT, 4.0f);
        CHECK(l.state() == LS_RTK_PAUSED && !l.allowRtk() && l.pauseCount() == 1);
        CHECK(strstr(l.reason(), "paused for 15 min") && strstr(l.reason(), "4.0 m") && strstr(l.reason(), "0.8 m"));
        // during the pause the receiver falls back; the ladder stays paused for 15 min
        run(l, t, 899, true, false, M_SBAS, 0.7f);
        CHECK(l.state() == LS_RTK_PAUSED && !l.allowRtk());
        run(l, t, 2, true, false, M_SBAS, 0.7f);
        CHECK(l.allowRtk() && l.state() == LS_RTK_TRYING);
        // fails again -> next pause is 30 min
        run(l, t, 301, true, true, M_RTK_FLOAT, 4.0f);
        CHECK(l.state() == LS_RTK_PAUSED && strstr(l.reason(), "paused for 30 min"));
    }
    // 4. Poor float but better than the fallback: keeps RTK
    {
        Ladder l;
        uint32_t t = 1000;
        run(l, t, 60, false, false, M_SBAS, 1.2f);
        run(l, t, 900, true, true, M_RTK_FLOAT, 0.9f);
        CHECK(l.state() == LS_RTK && l.allowRtk() && l.pauseCount() == 0);
    }
    // 5. No alternative seen yet: compared with a typical SBAS result (1.5 m)
    {
        Ladder l;
        uint32_t t = 1000;
        run(l, t, 400, true, true, M_RTK_FLOAT, 1.2f);
        CHECK(l.allowRtk());
        Ladder l2;
        t = 1000;
        run(l2, t, 400, true, true, M_RTK_FLOAT, 3.0f);
        CHECK(!l2.allowRtk() && strstr(l2.reason(), "SBAS typically gives 1.5 m"));
    }
    // 6. Corrections arrive but the receiver never reaches RTK (wrong format, distant base)
    {
        Ladder l;
        uint32_t t = 1000;
        run(l, t, 301, true, true, M_DGNSS, 1.0f);
        CHECK(l.state() == LS_RTK_PAUSED && strstr(l.reason(), "no RTK solution"));
        // a source that delivers nothing (caster down) is not paused, it just waits
        Ladder w;
        t = 1000;
        run(w, t, 1200, true, false, M_SBAS, 1.0f);
        CHECK(w.allowRtk() && w.state() == LS_RTK_TRYING && strstr(w.reason(), "waiting for corrections"));
    }
    // 7. Float converging slowly but reaching 0.5 m within the trial: no pause; good float later lapses briefly
    {
        Ladder l;
        uint32_t t = 1000;
        run(l, t, 200, true, true, M_RTK_FLOAT, 2.0f);
        run(l, t, 200, true, true, M_RTK_FLOAT, 0.3f);
        run(l, t, 200, true, true, M_RTK_FLOAT, 2.0f); // within a trial period of the last good epoch
        CHECK(l.allowRtk() && l.pauseCount() == 0);
    }
    // 8. Source disappears (Wi-Fi out of range): no source, trial restarts when it comes back
    {
        Ladder l;
        uint32_t t = 1000;
        run(l, t, 200, true, true, M_RTK_FLOAT, 3.0f);
        run(l, t, 30, false, false, M_SBAS, 1.0f);
        CHECK(l.state() == LS_NO_SOURCE);
        run(l, t, 200, true, true, M_RTK_FLOAT, 3.0f);
        CHECK(l.allowRtk()); // a new 5 min trial
    }
    // 9. HAS only mode
    {
        Ladder l;
        LadderConfig c;
        c.rtkAllowed = false;
        l.configure(c);
        uint32_t t = 1000;
        run(l, t, 3, true, false, M_HAS_CONV, 1.0f);
        CHECK(l.state() == LS_HAS_ONLY && !l.allowRtk());
    }
    // 10. Pause doubling is cleared by a minute of RTK fixed; it never exceeds the maximum
    {
        Ladder l;
        uint32_t t = 1000;
        for (int k = 0; k < 4; k++)
        {
            run(l, t, 301, true, true, M_RTK_FLOAT, 5.0f);
            run(l, t, 3601, true, false, M_SBAS, 1.0f);
        }
        run(l, t, 301, true, true, M_RTK_FLOAT, 5.0f);
        CHECK(strstr(l.reason(), "paused for 60 min"));
        run(l, t, 3601, true, false, M_SBAS, 1.0f);
        run(l, t, 120, true, true, M_RTK_FIXED, 0.02f);
        run(l, t, 1000, false, false, M_SBAS, 1.0f);
        run(l, t, 301, true, true, M_RTK_FLOAT, 5.0f);
        CHECK(strstr(l.reason(), "paused for 15 min"));
    }
    // 11. millis() wrap-around during a pause
    {
        Ladder l;
        uint32_t t = 0xFFFFFFFFu - 400000u;
        run(l, t, 301, true, true, M_RTK_FLOAT, 5.0f);
        CHECK(!l.allowRtk());
        run(l, t, 899, true, false, M_SBAS, 1.0f);
        CHECK(!l.allowRtk());
        run(l, t, 2, true, false, M_SBAS, 1.0f);
        CHECK(l.allowRtk());
    }
    // 12. $PRFSM sentence
    {
        char b[160];
        CorrInfo c;
        c.live = true;
        c.age = 1.0f;
        c.station = 1234;
        size_t n = prfsmSentence(b, sizeof(b), LS_RTK, M_RTK_FIXED, c, 0.0141f, 0.0252f);
        CHECK(n > 0 && strncmp(b, "$PRFSM,RTK,FIX,live,1.0,1234,0.014,0.025*", 41) == 0);
        CHECK(nmeaChecksumOk(b, n));
        CorrInfo none;
        n = prfsmSentence(b, sizeof(b), LS_NO_SOURCE, M_GNSS, none, -1, -1);
        CHECK(n > 0 && strncmp(b, "$PRFSM,NOSRC,GNSS,none,,,,*", 27) == 0);
        CHECK(nmeaChecksumOk(b, n));
    }
}

// UBX frames: checksum of a known frame, VALSET layout, parser ACK / NAK / MON-VER, NMEA mixed in
static void testUblox()
{
    uint8_t f[64];
    // MON-VER poll: B5 62 0A 04 00 00 0E 34 (u-blox Interface Description)
    size_t n = ubxMonVerPoll(f, sizeof(f));
    CHECK(n == 8 && f[6] == 0x0E && f[7] == 0x34);
    // VALSET of a 1-byte key: payload 9 bytes, layers RAM+BBR
    UbxKey k{0x209100bb, 1, "GGA"};
    n = ubxValset(k, f, sizeof(f));
    CHECK(n == 17 && f[2] == 0x06 && f[3] == 0x8A && f[4] == 9 && f[7] == 0x03 && f[10] == 0xbb && f[13] == 0x20 && f[14] == 1);
    // 4-byte key (baud)
    UbxKey kb{UBX_KEY_UART1_BAUD, 115200, "baud"};
    n = ubxValset(kb, f, sizeof(f));
    CHECK(n == 20 && f[14] == 0x00 && f[15] == 0xC2 && f[16] == 0x01 && f[17] == 0x00);
    // every setup key has a valid size
    for (int i = 0; i < UBX_SETUP_COUNT; i++)
        CHECK(ubxValset(UBX_SETUP[i], f, sizeof(f)) > 0);

    UbxParser p;
    // ACK-ACK for CFG-VALSET, with NMEA bytes and noise around it
    uint8_t ackPl[2] = {0x06, 0x8A};
    size_t an = ubxFrame(0x05, 0x01, ackPl, 2, f, sizeof(f));
    const char *junk = "$GNGGA,1*00\r\n\xB5";
    int acks = 0;
    for (const char *c = junk; *c; c++)
        p.feed((uint8_t)*c);
    for (size_t i = 0; i < an; i++)
        if (p.feed(f[i]) == UBX_ACK && p.ackedClass() == 0x06 && p.ackedId() == 0x8A)
            acks++;
    CHECK(acks == 1);
    // NAK
    an = ubxFrame(0x05, 0x00, ackPl, 2, f, sizeof(f));
    int naks = 0;
    for (size_t i = 0; i < an; i++)
        if (p.feed(f[i]) == UBX_NAK)
            naks++;
    CHECK(naks == 1);
    // corrupted checksum: no event, counted as an error
    f[an - 1] ^= 0xFF;
    int any = 0;
    for (size_t i = 0; i < an; i++)
        if (p.feed(f[i]) != UBX_NONE)
            any++;
    CHECK(any == 0 && p.errors() >= 1);
    // MON-VER with extensions
    uint8_t mv[40 + 3 * 30] = {0};
    memcpy(mv, "EXT CORE 1.00 (a59682)", 22);
    memcpy(mv + 30, "00190000", 8);
    memcpy(mv + 40, "FWVER=HPS 1.30", 14);
    memcpy(mv + 70, "PROTVER=33.30", 13);
    memcpy(mv + 100, "MOD=ZED-F9R", 11);
    static uint8_t big[200];
    size_t vn = ubxFrame(0x0A, 0x04, mv, sizeof(mv), big, sizeof(big));
    int ver = 0;
    for (size_t i = 0; i < vn; i++)
        if (p.feed(big[i]) == UBX_MONVER)
            ver++;
    CHECK(ver == 1);
    CHECK(std::string(p.version()) == "HPS 1.30, protocol 33.30, ZED-F9R");
}

int main()
{
    testUblox();
    testLadder();
    testEvTrim();
    testSky();
    testHistory();
    testMethods();
    testNmea();
    testRtcm();
    testSelector();
    testNtrip();
    testRing();
    printf("%d checks, %d failures\n", checks, fails);
    return fails ? 1 : 0;
}
