// eventlog.cpp - see eventlog.h
#include "eventlog.h"
#include "evtrim.h"
#include "shared.h"
#include <LittleFS.h>

#ifndef POSTCARDRTK_VERSION
#define POSTCARDRTK_VERSION "0.1.0"
#endif

static const char *F_LOG = "/events.log", *F_TMP = "/events.tmp", *F_SESS = "/evsess";
static uint32_t session = 0, seq = 0, lastFlushMs = 0;
static String pending;

void eventsSetup()
{
    if (File f = LittleFS.open(F_SESS, "r"))
    {
        session = f.readString().toInt();
        f.close();
    }
    session++;
    if (File f = LittleFS.open(F_SESS, "w"))
    {
        f.print(session);
        f.close();
    }
    pending = "S\t" + String(session) + "\t" + String(POSTCARDRTK_VERSION) + "\n";
    lastFlushMs = millis();
}

void eventAdd(const char *msg)
{
    char clean[128];
    size_t n = 0;
    for (const char *p = msg; *p && n < sizeof(clean) - 1; p++)
        clean[n++] = (*p == '\t' || *p == '\n' || *p == '\r') ? ' ' : *p;
    clean[n] = 0;
    const int64_t utc = currentUtc();
    pending += "E\t" + String(session) + "\t" + String(millis() / 1000) + "\t" + (utc > 0 ? String((long)utc) : String("-")) + "\t" +
               clean + "\n";
    seq++;
    if (pending.length() > 2048)
        eventsFlush();
}

void eventsFlush()
{
    if (!pending.length())
        return;
    size_t size = 0;
    if (File f = LittleFS.open(F_LOG, "r"))
    {
        size = f.size();
        f.close();
    }
    if (size + pending.length() <= EV_MAX_BYTES)
    {
        if (File f = LittleFS.open(F_LOG, "a"))
        {
            f.print(pending);
            f.close();
        }
    }
    else
    {
        // Full: read, trim (earlier sessions first), write to a new file, then swap
        std::string all;
        all.reserve(size + pending.length());
        if (File f = LittleFS.open(F_LOG, "r"))
        {
            char b[512];
            int k;
            while ((k = f.read((uint8_t *)b, sizeof(b))) > 0)
                all.append(b, k);
            f.close();
        }
        all.append(pending.c_str(), pending.length());
        std::string t = evTrim(all, session, EV_MAX_BYTES);
        if (File f = LittleFS.open(F_TMP, "w"))
        {
            f.write((const uint8_t *)t.data(), t.size());
            f.close();
            LittleFS.remove(F_LOG);
            LittleFS.rename(F_TMP, F_LOG);
        }
    }
    pending = "";
    lastFlushMs = millis();
}

void eventsLoop()
{
    if (pending.length() && millis() - lastFlushMs > 30000)
        eventsFlush();
}

uint32_t eventsSeq() { return seq; }
uint32_t eventsSession() { return session; }

void eventsText(String &out)
{
    out = "C\t" + String(session) + "\t" + String(seq) + "\n";
    if (File f = LittleFS.open(F_LOG, "r"))
    {
        out.reserve(f.size() + pending.length() + 32);
        char b[512];
        int k;
        while ((k = f.read((uint8_t *)b, sizeof(b))) > 0)
            out.concat(b, k);
        f.close();
    }
    out += pending;
}
