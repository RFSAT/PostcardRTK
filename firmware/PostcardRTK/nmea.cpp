// nmea.cpp - see nmea.h
#include "nmea.h"
#include <stdlib.h>
#include <cstdio>
#include <cstdlib>
#include <cstring>

static int hexVal(char c)
{
    if (c >= '0' && c <= '9')
        return c - '0';
    if (c >= 'A' && c <= 'F')
        return c - 'A' + 10;
    if (c >= 'a' && c <= 'f')
        return c - 'a' + 10;
    return -1;
}

uint8_t nmeaChecksum(const char *body, size_t len)
{
    uint8_t c = 0;
    for (size_t i = 0; i < len; i++)
        c ^= (uint8_t)body[i];
    return c;
}

bool nmeaChecksumOk(const char *s, size_t len)
{
    while (len && (s[len - 1] == '\r' || s[len - 1] == '\n'))
        len--;
    if (len < 4 || s[0] != '$' || s[len - 3] != '*')
        return false;
    int h = hexVal(s[len - 2]), l = hexVal(s[len - 1]);
    if (h < 0 || l < 0)
        return false;
    return nmeaChecksum(s + 1, len - 4) == (uint8_t)(h * 16 + l);
}

size_t nmeaFinish(const char *in, char *out, size_t outSize)
{
    size_t len = strlen(in);
    while (len && (in[len - 1] == '\r' || in[len - 1] == '\n' || in[len - 1] == ' '))
        len--;
    if (len + 6 > outSize || len == 0)
        return 0;
    memcpy(out, in, len);
    bool hasStar = (len >= 3 && in[len - 3] == '*');
    if (!hasStar && in[0] == '$')
    {
        static const char hx[] = "0123456789ABCDEF";
        uint8_t c = nmeaChecksum(in + 1, len - 1);
        out[len++] = '*';
        out[len++] = hx[c >> 4];
        out[len++] = hx[c & 15];
    }
    out[len++] = '\r';
    out[len++] = '\n';
    out[len] = 0;
    return len;
}

bool NmeaAssembler::feed(uint8_t c)
{
    if (c == '$')
    {
        _in = true;
        _len = 0;
    }
    if (!_in)
        return false;
    if (c == '\n' || c == '\r')
    {
        _in = false;
        if (_len < 4)
            return false;
        _buf[_len] = 0;
        if (nmeaChecksumOk(_buf, _len))
        {
            memcpy(_out, _buf, _len);
            _out[_len++] = '\r';
            _out[_len++] = '\n';
            _out[_len] = 0;
            _outLen = _len;
            _good++;
            return true;
        }
        _bad++;
        return false;
    }
    if (c < 0x20 || c > 0x7e || _len >= NMEA_MAX)
    {
        // Binary data or an over-long line: drop it
        _in = false;
        _bad++;
        return false;
    }
    _buf[_len++] = (char)c;
    return false;
}

size_t nmeaMakeGga(double lat, double lon, float alt, uint32_t secOfDay, char *out, size_t outSize)
{
    char ns = lat < 0 ? 'S' : 'N', ew = lon < 0 ? 'W' : 'E';
    lat = lat < 0 ? -lat : lat;
    lon = lon < 0 ? -lon : lon;
    int latD = (int)lat, lonD = (int)lon;
    double latM = (lat - latD) * 60.0, lonM = (lon - lonD) * 60.0;
    secOfDay %= 86400;
    char body[120];
    snprintf(body, sizeof(body), "$GPGGA,%02u%02u%02u.00,%02d%010.7f,%c,%03d%010.7f,%c,1,12,1.0,%.3f,M,0.000,M,,",
             (unsigned)(secOfDay / 3600), (unsigned)(secOfDay / 60 % 60), (unsigned)(secOfDay % 60), latD, latM, ns,
             lonD, lonM, ew, (double)alt);
    return nmeaFinish(body, out, outSize);
}

bool nmeaSentenceId(const char *s, size_t len, char *id, size_t idSize)
{
    if (len < 7 || s[0] != '$' || idSize < 4)
        return false;
    size_t end = 1;
    while (end < len && s[end] != ',' && s[end] != '*')
        end++;
    const char *start = s + 1;
    size_t n = end - 1;
    if (s[1] != 'P') // talker (2 chars) + type
    {
        if (n < 4)
            return false;
        start += 2;
        n -= 2;
    }
    if (n >= idSize)
        n = idSize - 1;
    memcpy(id, start, n);
    id[n] = 0;
    return n > 0;
}

bool nmeaIsReply(const char *s, size_t len)
{
    if (len < 8 || strncmp(s, "$PQTM", 5) != 0)
        return false;
    const char *comma = (const char *)memchr(s, ',', len);
    if (!comma)
        return false;
    size_t idLen = comma - (s + 1), rest = len - (comma + 1 - s);
    static const char *const ids[] = {"PQTMVER", "PQTMVERNO", "PQTMUNIQID", "PQTMSN"};
    for (const char *id : ids)
        if (idLen == strlen(id) && strncmp(s + 1, id, idLen) == 0)
            return true;
    return (rest >= 2 && strncmp(comma + 1, "OK", 2) == 0 && (rest == 2 || comma[3] == ',' || comma[3] == '*')) ||
           (rest >= 5 && strncmp(comma + 1, "ERROR", 5) == 0);
}

// Split the fields after the sentence id into f[] (pointers into buf); returns the field count
static int nmeaFields(const char *s, size_t len, char *buf, size_t bufSize, const char **f, int maxF)
{
    size_t n = len < bufSize - 1 ? len : bufSize - 1;
    memcpy(buf, s, n);
    buf[n] = 0;
    char *star = strchr(buf, '*');
    if (star)
        *star = 0;
    char *p = strchr(buf, ',');
    int k = 0;
    while (p && k < maxF)
    {
        *p++ = 0;
        f[k++] = p;
        p = strchr(p, ',');
    }
    return k;
}

static bool numField(const char *f, float &v)
{
    if (!f || !*f)
        return false;
    char *end;
    v = strtof(f, &end);
    return end != f;
}

bool nmeaParseGst(const char *s, size_t len, float &latStd, float &lonStd, float &altStd)
{
    char buf[NMEA_MAX + 1];
    const char *f[12];
    int k = nmeaFields(s, len, buf, sizeof(buf), f, 12);
    return k >= 8 && numField(f[5], latStd) && numField(f[6], lonStd) && numField(f[7], altStd);
}

bool nmeaParseEpe(const char *s, size_t len, float &north, float &east, float &down, float &h2d, float &v3d)
{
    char buf[NMEA_MAX + 1];
    const char *f[8];
    int k = nmeaFields(s, len, buf, sizeof(buf), f, 8);
    return k >= 6 && numField(f[1], north) && numField(f[2], east) && numField(f[3], down) && numField(f[4], h2d) &&
           numField(f[5], v3d);
}

bool nmeaParsePppNav(const char *s, size_t len, PppNav &out)
{
    char buf[NMEA_MAX + 1];
    const char *f[48];
    int k = nmeaFields(s, len, buf, sizeof(buf), f, 48);
    if (k < 28)
        return false;
    float v;
    out = PppNav();
    if (numField(f[10], v)) out.sol = (int)v;
    numField(f[18], out.latStd);
    numField(f[19], out.lonStd);
    numField(f[20], out.altStd);
    if (numField(f[23], v)) out.diffId = (int)v;
    numField(f[24], out.diffAge);
    if (numField(f[27], v)) out.satUsed = (int)v;
    return out.sol >= 0;
}

bool nmeaIsGga(const char *s, size_t len)
{
    return len > 7 && s[0] == '$' && s[3] == 'G' && s[4] == 'G' && s[5] == 'A' && s[6] == ',';
}

// Copy field n (0 = sentence id) into buf; returns its length or -1 when missing
static int field(const char *s, size_t len, int n, char *buf, size_t bufSize)
{
    int idx = 0;
    size_t start = 0;
    for (size_t i = 0; i <= len; i++)
    {
        char c = (i < len) ? s[i] : ',';
        if (c == ',' || c == '*' || c == '\r' || c == '\n')
        {
            if (idx == n)
            {
                size_t l = i - start;
                if (l >= bufSize)
                    l = bufSize - 1;
                memcpy(buf, s + start, l);
                buf[l] = 0;
                return (int)l;
            }
            idx++;
            start = i + 1;
            if (c == '*')
                break;
        }
    }
    return -1;
}

static double nmeaDeg(const char *v, const char *hemi, int degDigits)
{
    if (!v[0])
        return 0;
    char d[4] = {0};
    memcpy(d, v, degDigits);
    double deg = atof(d) + atof(v + degDigits) / 60.0;
    if (hemi[0] == 'S' || hemi[0] == 'W')
        deg = -deg;
    return deg;
}

bool nmeaParseGga(const char *s, size_t len, GgaInfo &g)
{
    if (!nmeaIsGga(s, len))
        return false;
    char f[24], h[4];
    g = GgaInfo();
    if (field(s, len, 1, f, sizeof(f)) >= 0)
        strncpy(g.utc, f, sizeof(g.utc) - 1);
    if (field(s, len, 2, f, sizeof(f)) > 4 && field(s, len, 3, h, sizeof(h)) >= 0)
        g.lat = nmeaDeg(f, h, 2);
    if (field(s, len, 4, f, sizeof(f)) > 5 && field(s, len, 5, h, sizeof(h)) >= 0)
        g.lon = nmeaDeg(f, h, 3);
    if (field(s, len, 6, f, sizeof(f)) > 0)
        g.quality = atoi(f);
    if (field(s, len, 7, f, sizeof(f)) > 0)
        g.sats = atoi(f);
    if (field(s, len, 8, f, sizeof(f)) > 0)
        g.hdop = (float)atof(f);
    if (field(s, len, 9, f, sizeof(f)) > 0)
        g.alt = (float)atof(f);
    if (field(s, len, 13, f, sizeof(f)) > 0)
        g.age = (float)atof(f);
    if (field(s, len, 14, f, sizeof(f)) > 0)
        g.station = atoi(f);
    g.valid = true;
    return true;
}
