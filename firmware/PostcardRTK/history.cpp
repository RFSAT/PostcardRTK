// history.cpp - see history.h. Records arrive from core 1 through histToCore0, are buffered in RAM
// and appended to flash every 5 minutes (and at shutdown / restart / download). When the current
// file reaches HIST_FILE_MAX it becomes the old one and a new file starts, so the oldest data goes first.
#include "history.h"
#include "shared.h"
#include "diag.h"
#include <LittleFS.h>

// Format 2 records (72 bytes); older format files are removed at start-up
static const char *F_NEW = "/hist2a.bin", *F_OLD = "/hist2b.bin";
static const int BUF_N = 40;
static HistRec buf[BUF_N];
static int bufN = 0;
static uint32_t lastFlushMs = 0;
static const uint32_t FLUSH_MS = 5u * 60u * 1000u;

static uint32_t fileRecs(const char *p)
{
    if (!LittleFS.exists(p))
        return 0;
    File f = LittleFS.open(p, "r");
    uint32_t n = f ? f.size() / sizeof(HistRec) : 0;
    f.close();
    return n;
}

static void add(const HistRec &r);
static void drain()
{
    HistRec r;
    while (histToCore0.available() >= sizeof(HistRec))
    {
        histToCore0.read((uint8_t *)&r, sizeof(r));
        add(r);
    }
}

static SkySnap lastSky; // previous run's sky, for the Sky tab until the receiver locks
const SkySnap &skyLastKnown() { return lastSky; }

// Save the live sky with the time of the newest dated record, so the next start can show it
static void skySave()
{
    int64_t utc = -1;
    for (int i = bufN - 1; i >= 0 && utc < 0; i--)
        utc = histEpoch(buf[i]);
    if (utc < 0)
        return; // no date yet
    static SkyTable t;
    static SkySnap snap;
    getSky(t);
    if (skySnapshot(t, millis(), utc, snap) == 0)
        return;
    File f = LittleFS.open("/sky.bin", "w");
    if (f)
    {
        f.write((const uint8_t *)&snap, sizeof(snap));
        f.close();
    }
}

void historyFlush()
{
    drain(); // include the records still on their way from core 1 (shutdown, restart, download)
    if (!bufN)
        return;
    skySave();
    if (fileRecs(F_NEW) * sizeof(HistRec) >= HIST_FILE_MAX)
    {
        LittleFS.remove(F_OLD);
        LittleFS.rename(F_NEW, F_OLD);
    }
    File f = LittleFS.open(F_NEW, "a");
    if (f)
    {
        f.write((const uint8_t *)buf, bufN * sizeof(HistRec));
        f.close();
    }
    bufN = 0;
    lastFlushMs = millis();
}

static void add(const HistRec &r)
{
    if (bufN >= BUF_N)
        historyFlush();
    buf[bufN++] = r;
}

void historySetup()
{
    if (File f = LittleFS.open("/sky.bin", "r"))
    {
        if (f.read((uint8_t *)&lastSky, sizeof(lastSky)) != sizeof(lastSky) || lastSky.magic != SKY_SNAP_MAGIC ||
            lastSky.n > SKY_MAX_SATS)
            lastSky = SkySnap();
        f.close();
    }
    for (const char *old : {"/hist0.bin", "/hist1.bin"})
        if (LittleFS.exists(old))
            LittleFS.remove(old);
    HistRec r{};
    r.ymd = HIST_RESTART;
    r.upSec = millis() / 1000;
    add(r);
    lastFlushMs = millis();
}

void historyLoop()
{
    diagC0 = 81;
    drain();
    diagC0 = 83;
    if (bufN && millis() - lastFlushMs >= FLUSH_MS)
    {
        diagC0 = 84;
        historyFlush();
    }
}

bool historyClear()
{
    bufN = 0;
    bool ok = true;
    if (LittleFS.exists(F_OLD))
        ok &= LittleFS.remove(F_OLD);
    if (LittleFS.exists(F_NEW))
        ok &= LittleFS.remove(F_NEW);
    return ok;
}

uint32_t historyRecords() { return fileRecs(F_OLD) + fileRecs(F_NEW) + bufN; }
uint32_t historyBytes() { return (fileRecs(F_OLD) + fileRecs(F_NEW)) * sizeof(HistRec); }

void historyForEach(bool (*fn)(const HistRec &, void *), void *ctx)
{
    for (const char *p : {F_OLD, F_NEW})
    {
        if (!LittleFS.exists(p))
            continue;
        File f = LittleFS.open(p, "r");
        HistRec r;
        while (f && f.read((uint8_t *)&r, sizeof(r)) == sizeof(r))
            if (!fn(r, ctx))
            {
                f.close();
                return;
            }
        f.close();
    }
    for (int i = 0; i < bufN; i++)
        if (!fn(buf[i], ctx))
            return;
}

void historyBackward(bool (*fn)(const HistRec &, void *), void *ctx)
{
    for (int i = bufN - 1; i >= 0; i--)
        if (!fn(buf[i], ctx))
            return;
    static HistRec chunk[16];
    for (const char *p : {F_NEW, F_OLD})
    {
        uint32_t n = fileRecs(p);
        if (!n)
            continue;
        File f = LittleFS.open(p, "r");
        while (f && n)
        {
            uint32_t k = n < 16 ? n : 16;
            n -= k;
            f.seek(n * sizeof(HistRec));
            if (f.read((uint8_t *)chunk, k * sizeof(HistRec)) != k * sizeof(HistRec))
                break;
            for (int i = (int)k - 1; i >= 0; i--)
                if (!fn(chunk[i], ctx))
                {
                    f.close();
                    return;
                }
        }
        f.close();
    }
}

static bool dated(const HistRec &r) { return r.ymd && r.ymd != HIST_RESTART; }
struct FirstCtx
{
    uint32_t y = 0, m = 0;
};
static bool firstCb(const HistRec &r, void *c)
{
    if (!dated(r))
        return true;
    ((FirstCtx *)c)->y = r.ymd;
    ((FirstCtx *)c)->m = r.utcMs;
    return false; // stop at the first dated record
}
void historyInfo(uint32_t &fy, uint32_t &fm, uint32_t &ly, uint32_t &lm)
{
    FirstCtx x;
    historyForEach(firstCb, &x);
    fy = x.y;
    fm = x.m;
    ly = lm = 0;
    for (int i = bufN - 1; i >= 0; i--) // newest: the RAM buffer first, then the end of the newest file
        if (dated(buf[i]))
        {
            ly = buf[i].ymd;
            lm = buf[i].utcMs;
            return;
        }
    for (const char *p : {F_NEW, F_OLD})
    {
        uint32_t n = fileRecs(p);
        if (!n)
            continue;
        File f = LittleFS.open(p, "r");
        HistRec r;
        for (uint32_t k = n; k > 0 && n - k < 64; k--) // look back at most 64 records (restarts without a fix)
        {
            f.seek((k - 1) * sizeof(HistRec));
            if (f.read((uint8_t *)&r, sizeof(r)) == sizeof(r) && dated(r))
            {
                ly = r.ymd;
                lm = r.utcMs;
                f.close();
                return;
            }
        }
        f.close();
    }
}
