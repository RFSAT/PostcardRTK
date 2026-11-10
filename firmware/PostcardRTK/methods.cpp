// methods.cpp - see methods.h
#include "methods.h"
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

const char *methodName(Method m)
{
    switch (m)
    {
    case M_GNSS: return "GNSS only";
    case M_SBAS: return "SBAS (EGNOS)";
    case M_DGNSS: return "DGNSS (RTCM)";
    case M_RTK_FLOAT: return "RTK float";
    case M_RTK_FIXED: return "RTK fixed";
    case M_HAS_CONV: return "HAS converging";
    case M_HAS_DONE: return "HAS converged";
    case M_DR: return "Dead reckoning";
    default: return "No fix";
    }
}

const char *groupName(MethodGroup g)
{
    switch (g)
    {
    case G_RTK: return "RTK (RTCM)";
    case G_HAS: return "Galileo HAS";
    case G_SBAS: return "SBAS (EGNOS)";
    default: return "GNSS only";
    }
}

int methodGroup(Method m)
{
    switch (m)
    {
    case M_RTK_FIXED:
    case M_RTK_FLOAT:
    case M_DGNSS: return G_RTK;
    case M_HAS_CONV:
    case M_HAS_DONE: return G_HAS;
    case M_SBAS: return G_SBAS;
    case M_GNSS: return G_GNSS;
    default: return -1;
    }
}

Method classifyMethod(const GgaInfo &g, const PppNav *ppp, bool rtcmActive)
{
    if (!g.valid)
        return M_NONE;
    const int q = g.quality, st = g.station;
    const bool has = st == 9002 || (ppp && ppp->diffId == 9002 && (ppp->sol == 6 || ppp->sol == 7));
    if (has && q != 0)
        return ((ppp && ppp->sol == 7) || (!ppp && q == 4)) ? M_HAS_DONE : M_HAS_CONV;
    switch (q)
    {
    case 4: return M_RTK_FIXED;
    case 5: return M_RTK_FLOAT;
    case 2:
    {
        // SBAS fixes report no station, or the EGNOS satellite number (PRN 120-158) as the station
        const bool sbasPrn = st >= 120 && st <= 158;
        if (st >= 0 && !(sbasPrn && !rtcmActive))
            return M_DGNSS;
        return M_SBAS;
    }
    case 1: return M_GNSS;
    case 6: return M_DR;
    default: return M_NONE;
    }
}

CorrInfo correctionsFor(Method m, const GgaInfo &g, const PppNav *ppp, bool rtcmActive)
{
    CorrInfo c;
    switch (m)
    {
    case M_RTK_FIXED:
    case M_RTK_FLOAT:
    case M_DGNSS:
        c.live = rtcmActive;
        c.age = g.age;
        c.station = g.station;
        break;
    case M_HAS_CONV:
    case M_HAS_DONE:
        c.age = (ppp && ppp->diffAge >= 0) ? ppp->diffAge : g.age;
        c.live = c.age >= 0 && c.age < 60; // HAS corrections arrive from the satellites every few seconds
        c.station = 9002;
        break;
    case M_SBAS:
        c.live = true;
        c.age = g.age;
        c.station = (g.station >= 120 && g.station <= 158) ? g.station : -1;
        break;
    default:
        break;
    }
    return c;
}

float rmcSpeed(const char *s, size_t len)
{
    // field 7 of $xxRMC: speed over ground in knots
    int f = 0;
    size_t i = 0;
    for (; i < len && f < 7; i++)
        if (s[i] == ',')
            f++;
    if (f < 7 || i >= len || s[i] == ',' || s[i] == '*')
        return -1;
    char b[16];
    size_t n = 0;
    while (i < len && s[i] != ',' && s[i] != '*' && n < sizeof(b) - 1)
        b[n++] = s[i++];
    b[n] = 0;
    return (float)(atof(b) * 0.514444);
}

void refError(const RefPoint &r, double lat, double lon, float alt, float &errH, float &errV)
{
    if (!r.set)
    {
        errH = errV = -1;
        return;
    }
    const double R = 6378137.0, d2r = M_PI / 180.0;
    const double dN = (lat - r.lat) * d2r * R;
    const double dE = (lon - r.lon) * d2r * R * cos(r.lat * d2r);
    errH = (float)sqrt(dN * dN + dE * dE);
    errV = fabsf(alt - r.alt);
}

bool MethodTracker::update(uint32_t now, const GgaInfo &g, Method m, float estH, float estV, const RefPoint &ref,
                           const CorrInfo &c, float speed)
{
    if (m != _cur)
    {
        int og = methodGroup(_cur), ng = methodGroup(m);
        if (ng >= 0 && (og != ng || !_st[ng].seen || now - _st[ng].lastMs > 5000))
        {
            _st[ng].startMs = now;
            _st[ng].bestEstH = -1;
        }
        _cur = m;
        _curSince = now;
    }
    float errH = -1, errV = -1;
    if (m != M_NONE)
        refError(ref, g.lat, g.lon, g.alt, errH, errV);
    const int grp = methodGroup(m);
    if (grp >= 0)
    {
        MethodStat &s = _st[grp];
        if (!s.seen)
            s.startMs = now;
        s.seen = true;
        s.lastMs = now;
        s.last = m;
        s.estH = estH;
        s.estV = estV;
        s.errH = errH;
        s.errV = errV;
        s.corr = c;
        if (estH >= 0 && (s.bestEstH < 0 || estH < s.bestEstH))
            s.bestEstH = estH;
    }

    // Movement with hysteresis, so stops at traffic lights do not flip it every second
    _speed = speed;
    if (speed < 0)
        _fastSince = _slowSince = 0;
    else if (speed > 0.5f)
    {
        _slowSince = 0;
        if (!_fastSince)
            _fastSince = now ? now : 1;
        if (now - _fastSince >= 3000)
            _moving = true;
    }
    else
    {
        _fastSince = 0;
        if (speed < 0.3f)
        {
            if (!_slowSince)
                _slowSince = now ? now : 1;
            if (now - _slowSince >= 10000)
                _moving = false;
        }
    }

    // Trace, one sample every 10 s
    if (!_traceStarted || now - _lastTraceMs >= TRACE_STEP_MS)
    {
        _traceStarted = true;
        _lastTraceMs = now;
        TraceSample &t = _tr[_traceHead];
        t.tSec = now / 1000;
        t.method = m;
        t.estH = estH;
        t.errH = errH;
        _traceHead = (_traceHead + 1) % TRACE_N;
        if (_traceN < TRACE_N)
            _traceN++;
        return true;
    }
    return false;
}

static void numOrEmpty(char *o, size_t n, float v, int dec)
{
    if (v < 0)
        o[0] = 0;
    else
        snprintf(o, n, "%.*f", dec, v);
}

size_t histCsvLine(const HistRec &r, char *out, size_t n)
{
    if (r.ymd == HIST_RESTART)
        return (size_t)snprintf(out, n, ",,%lu,restart,,,,,,,,,,,,,,\r\n", (unsigned long)r.upSec);
    char d[16] = "", t[16] = "", a[6][16];
    if (r.ymd)
        snprintf(d, sizeof(d), "%04lu-%02lu-%02lu", (unsigned long)(r.ymd / 10000), (unsigned long)(r.ymd / 100 % 100), (unsigned long)(r.ymd % 100));
    unsigned long ms = r.utcMs;
    snprintf(t, sizeof(t), "%02lu:%02lu:%02lu.%01lu", ms / 3600000, ms / 60000 % 60, ms / 1000 % 60, ms / 100 % 10);
    numOrEmpty(a[0], 16, r.estH, 3);
    numOrEmpty(a[1], 16, r.estV, 3);
    numOrEmpty(a[2], 16, r.errH, 3);
    numOrEmpty(a[3], 16, r.errV, 3);
    numOrEmpty(a[4], 16, r.corrAge, 1);
    numOrEmpty(a[5], 16, r.speed, 2);
    const Method m = (Method)r.method;
    const int g = methodGroup(m);
    const char *corr = (g < 0 || g == G_GNSS) ? "none" : (r.flags & HF_LIVE) ? "live" : "stale";
    char st[8] = "";
    if (r.station >= 0)
        snprintf(st, sizeof(st), "%d", r.station);
    char la[20] = "", lo[20] = "", al[16] = "";
    if (m != M_NONE)
    {
        snprintf(la, sizeof(la), "%.9f", r.lat);
        snprintf(lo, sizeof(lo), "%.9f", r.lon);
        snprintf(al, sizeof(al), "%.3f", r.alt);
    }
    return (size_t)snprintf(out, n, "%s,%s,%lu,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%u,%.2f\r\n", d, t, (unsigned long)r.upSec,
                            methodName(m), corr, a[4], st, a[0], a[1], a[2], a[3], la, lo, al, a[5],
                            r.speed < 0 ? "" : (r.flags & HF_MOVING) ? "yes" : "no", r.sats, r.hdop);
}

int64_t histEpoch(const HistRec &r)
{
    if (!r.ymd || r.ymd == HIST_RESTART)
        return -1;
    // days from civil date (Howard Hinnant's algorithm)
    int y = (int)(r.ymd / 10000), m = (int)(r.ymd / 100 % 100), d = (int)(r.ymd % 100);
    y -= m <= 2;
    const int era = (y >= 0 ? y : y - 399) / 400;
    const unsigned yoe = (unsigned)(y - era * 400);
    const unsigned doy = (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1;
    const unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    const int64_t days = (int64_t)era * 146097 + (int64_t)doe - 719468;
    return days * 86400 + r.utcMs / 1000;
}

uint32_t rmcDate(const char *s, size_t len)
{
    // field 9 of $xxRMC
    int f = 0;
    size_t i = 0;
    for (; i < len && f < 9; i++)
        if (s[i] == ',')
            f++;
    if (f < 9 || i + 6 > len)
        return 0;
    for (int k = 0; k < 6; k++)
        if (s[i + k] < '0' || s[i + k] > '9')
            return 0;
    auto two = [&](int k) { return (uint32_t)((s[i + k] - '0') * 10 + (s[i + k + 1] - '0')); };
    return (2000 + two(4)) * 10000 + two(2) * 100 + two(0);
}

uint32_t ggaTimeMs(const char *u)
{
    if (strlen(u) < 6)
        return 0;
    auto two = [&](int k) { return (uint32_t)((u[k] - '0') * 10 + (u[k + 1] - '0')); };
    uint32_t ms = (two(0) * 3600 + two(2) * 60 + two(4)) * 1000;
    if (u[6] == '.')
        ms += (uint32_t)(atof(u + 6) * 1000 + 0.5);
    return ms;
}

const TraceSample &MethodTracker::trace(int i) const
{
    int start = (_traceHead - _traceN + TRACE_N) % TRACE_N;
    return _tr[(start + i) % TRACE_N];
}
