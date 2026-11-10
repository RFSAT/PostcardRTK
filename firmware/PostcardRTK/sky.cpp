// sky.cpp - see sky.h
#include "sky.h"
#include <cstdlib>
#include <cstring>

const char *skyBandName(char sys, int id)
{
    switch (sys)
    {
    case 'G':
    case 'J':
        return id == 1 ? "L1" : id == 6 ? "L2" : id == 8 ? "L5" : "?";
    case 'R':
        return id == 1 ? "G1" : id == 3 ? "G2" : "?";
    case 'E':
        return id == 7 ? "E1" : id == 1 ? "E5a" : id == 2 ? "E5b" : id == 5 ? "E6" : "?";
    case 'C':
        return id == 1 ? "B1I" : id == 3 ? "B1C" : id == 5 ? "B2a" : id == 6 ? "B2b" : id == 8 ? "B3I" : id == 11 ? "B2I" : "?";
    case 'I':
        return id == 1 ? "L5" : "?";
    case 'S':
        return "L1";
    default:
        return "?";
    }
}

int skyBandOrder(char sys, int id)
{
    switch (sys)
    {
    case 'G':
    case 'J':
        return id == 1 ? 0 : id == 6 ? 2 : 3;
    case 'R':
        return id == 1 ? 0 : 2;
    case 'E':
        return id == 7 ? 0 : id == 5 ? 1 : id == 2 ? 2 : 3; // E1 1575, E6 1278, E5b 1207, E5a 1176 MHz
    case 'C':
        return id == 1 || id == 3 ? 0 : id == 8 ? 1 : id == 6 || id == 11 ? 2 : 3; // B1, B3, B2b/B2I, B2a
    default:
        return 0;
    }
}

// Comma-separated fields of a sentence body ($ ... before *); fields point into s, not terminated
static int split(const char *s, size_t len, const char **f, size_t *fl, int maxF)
{
    size_t end = 0;
    while (end < len && s[end] != '*')
        end++;
    int n = 0;
    size_t st = 1;
    for (size_t i = 1; i <= end && n < maxF; i++)
        if (i == end || s[i] == ',')
        {
            f[n] = s + st;
            fl[n] = i - st;
            n++;
            st = i + 1;
        }
    return n;
}

static long num(const char *p, size_t n, long dflt, int base = 10)
{
    if (!n || n > 10)
        return dflt;
    char b[12];
    memcpy(b, p, n);
    b[n] = 0;
    char *e;
    long v = strtol(b, &e, base);
    return *e ? dflt : v;
}

static char talkerSys(const char *t)
{
    if (!strncmp(t, "GP", 2)) return 'G';
    if (!strncmp(t, "GL", 2)) return 'R';
    if (!strncmp(t, "GA", 2)) return 'E';
    if (!strncmp(t, "GB", 2) || !strncmp(t, "BD", 2)) return 'C';
    if (!strncmp(t, "GQ", 2)) return 'J';
    if (!strncmp(t, "GI", 2)) return 'I';
    return 0;
}

static char systemIdSys(long id)
{
    switch (id)
    {
    case 1: return 'G';
    case 2: return 'R';
    case 3: return 'E';
    case 4: return 'C';
    case 5: return 'J';
    case 6: return 'I';
    default: return 0;
    }
}

// NMEA satellite ID -> system and number in that system (GPS-talker IDs 33-64 are SBAS PRN 120-151)
static void normalise(char &sys, long &id)
{
    if (sys == 'G' && id >= 33 && id <= 64)
    {
        sys = 'S';
        id += 87;
    }
    else if (sys == 'R' && id >= 65 && id <= 96)
        id -= 64;
}

SkySat *SkyTable::find(char sys, uint16_t prn, bool create, uint32_t now)
{
    SkySat *freeSlot = nullptr, *oldest = &_s[0];
    for (auto &x : _s)
    {
        if (x.sys == sys && x.prn == prn)
            return &x;
        if (!x.sys && !freeSlot)
            freeSlot = &x;
        if (x.sys && now - x.seenMs > now - oldest->seenMs)
            oldest = &x;
    }
    if (!create)
        return nullptr;
    SkySat *s = freeSlot ? freeSlot : oldest;
    *s = SkySat();
    s->sys = sys;
    s->prn = prn;
    return s;
}

bool SkyTable::feed(const char *s, size_t len, uint32_t now)
{
    if (len < 7 || s[0] != '$')
        return false;
    const bool gsv = !strncmp(s + 3, "GSV,", 4), gsa = !strncmp(s + 3, "GSA,", 4);
    if (!gsv && !gsa)
        return false;
    const char *f[32];
    size_t fl[32];
    int n = split(s, len, f, fl, 32);

    if (gsv)
    {
        // $xxGSV,total,num,inView{,id,el,az,cn0},signalId
        char sys = talkerSys(s + 1);
        if (!sys || n < 4)
            return true;
        const int blocks = (n - 4) / 4; // the last field is the signal ID when (n-4) % 4 == 1
        const bool hasSig = (n - 4) % 4 == 1;
        const long sigId = hasSig ? num(f[n - 1], fl[n - 1], 1, 16) : 1;
        for (int b = 0; b < blocks; b++)
        {
            const int k = 4 + 4 * b;
            long id = num(f[k], fl[k], -1);
            if (id <= 0)
                continue;
            char sy = sys;
            normalise(sy, id);
            SkySat *x = find(sy, (uint16_t)id, true, now);
            x->seenMs = now;
            long el = num(f[k + 1], fl[k + 1], -1), az = num(f[k + 2], fl[k + 2], -1), cn = num(f[k + 3], fl[k + 3], 0);
            if (el >= 0 && el <= 90)
                x->el = (int8_t)el;
            if (az >= 0 && az < 360)
                x->az = (int16_t)az;
            SkySig *slot = nullptr;
            for (auto &g : x->sig)
                if (g.id == sigId)
                {
                    slot = &g;
                    break;
                }
            if (!slot)
                for (auto &g : x->sig)
                    if (!g.id || now - g.atMs > 5000)
                    {
                        slot = &g;
                        break;
                    }
            if (slot)
            {
                slot->id = (uint8_t)sigId;
                slot->cn0 = cn > 0 && cn < 100 ? (uint8_t)cn : 0;
                slot->atMs = now;
            }
        }
        _lastGsv = now;
        if (!_firstGsv)
            _firstGsv = now ? now : 1;
        return true;
    }

    // $xxGSA,mode,fix,id x12,pdop,hdop,vdop,systemId
    if (n < 18)
        return true;
    char sys = systemIdSys(num(f[n - 1], fl[n - 1], 0));
    if (!sys)
        sys = talkerSys(s + 1);
    if (!sys)
        return true;
    for (int k = 3; k < 15 && k < n; k++)
    {
        long id = num(f[k], fl[k], -1);
        if (id <= 0)
            continue;
        char sy = sys;
        normalise(sy, id);
        SkySat *x = find(sy, (uint16_t)id, false, now);
        if (x)
            x->usedMs = now;
    }
    _lastGsa = now;
    return true;
}

void SkyTable::expire(uint32_t now, uint32_t maxAge)
{
    for (auto &x : _s)
    {
        if (x.sys && now - x.seenMs > maxAge)
            x = SkySat();
        else if (x.sys)
            for (auto &g : x.sig)
                if (g.id && now - g.atMs > SKY_LIVE_MS)
                    g = SkySig(); // lost lock: no signal level any more, the position is kept
    }
}

int skySnapshot(const SkyTable &t, uint32_t now, int64_t utc, SkySnap &o)
{
    o.magic = SKY_SNAP_MAGIC;
    o.utc = utc;
    o.n = 0;
    for (int i = 0; i < t.count(); i++)
    {
        const SkySat &x = t.at(i);
        if (!x.sys || !SkyTable::live(x, now) || x.el < 0 || x.az < 0)
            continue;
        SkySnapSat &s = o.s[o.n++];
        s.sys = x.sys;
        s.prn = x.prn;
        s.el = (uint8_t)x.el;
        s.az = x.az;
        s.used = SkyTable::used(x, now) ? 1 : 0;
        s.pad = 0;
    }
    return (int)o.n;
}
