// ntrip.cpp - core 0: NTRIP client with ordered sources and failover
#include "ntrip.h"
#include "eventlog.h"
#include "config.h"
#include "ntrip_proto.h"
#include "shared.h"
#include "util.h"
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <time.h>

#ifndef POSTCARDRTK_VERSION
#define POSTCARDRTK_VERSION "0.1.0"
#endif

static const uint32_t CONNECT_TIMEOUT_MS = 10000;
static const uint32_t REPLY_TIMEOUT_MS = 10000;
static const uint32_t WAIT_FIX_MAX_MS = 60000;    // then connect without a position
static const uint32_t AGE_GRACE_MS = 30000;       // after stream start before age is judged
static const uint32_t AGE_BAD_HOLD_MS = 10000;    // age must stay bad this long

static SourceSelector sel;
static NtripState state = NtripState::Off;
static WiFiClient plainClient;
static WiFiClientSecure tlsClient;
static Client *client = nullptr;
static NtripResponse reply;
static ChunkDecoder chunks;
static RtcmFramer framer;
static uint32_t stateAt = 0, waitMs = 0, streamStart = 0, lastFrameMs = 0, lastGgaSent = 0, ageBadSince = 0;
static uint32_t bytesTotal = 0, streamBytes = 0, ggaSentCount = 0;
static char lastGgaText[100] = "";
static char lastEvent[96] = "starting";

// Small event log for the web page
static const int LOG_N = 24;
static String logLines[LOG_N];
static int logHead = 0, logCount = 0;

static void event(const char *fmt, ...)
{
    char msg[96];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(msg, sizeof(msg), fmt, ap);
    va_end(ap);
    strncpy(lastEvent, msg, sizeof(lastEvent) - 1);
    char line[120];
    uint32_t s = millis() / 1000;
    snprintf(line, sizeof(line), "%02lu:%02lu:%02lu %s", (unsigned long)(s / 3600), (unsigned long)(s / 60 % 60),
             (unsigned long)(s % 60), msg);
    logLines[logHead] = line;
    logHead = (logHead + 1) % LOG_N;
    if (logCount < LOG_N)
        logCount++;
    dbg("[ntrip] %s\r\n", msg);
    eventAdd(msg); // kept in flash, by session
}

void ntripEventLog(String &out)
{
    out = "";
    for (int i = 0; i < logCount; i++)
    {
        int idx = (logHead - logCount + i + LOG_N) % LOG_N;
        out += logLines[idx];
        out += "\n";
    }
}

const char *ntripStateName(NtripState s)
{
    switch (s)
    {
    case NtripState::Off: return "off";
    case NtripState::NoNetwork: return "no network";
    case NtripState::Waiting: return "waiting";
    case NtripState::WaitFix: return "waiting for position";
    case NtripState::Connecting: return "connecting";
    case NtripState::Reply: return "waiting for caster";
    case NtripState::Streaming: return "streaming";
    case NtripState::Stopped: return "stopped";
    case NtripState::Paused: return "paused by the method ladder";
    }
    return "?";
}

static void setState(NtripState s)
{
    state = s;
    stateAt = millis();
}

static const NtripSource &cur() { return cfg.src[sel.active()]; }

static void closeClient()
{
    if (client)
        client->stop();
    client = nullptr;
}

static void scheduleNext(const SelectorDecision &d)
{
    if (d.stopped)
    {
        event("all sources rejected the login or mount point; check the settings");
        setState(NtripState::Stopped);
        return;
    }
    if (d.switched)
        event("-> source %d %s (%s/%s) in %lus", d.source + 1, cfg.src[d.source].name.c_str(),
              cfg.src[d.source].host.c_str(), cfg.src[d.source].mount.c_str(), (unsigned long)(d.delayMs / 1000));
    else if (d.backoff)
        event("all sources failed, retry in %lus", (unsigned long)(d.delayMs / 1000));
    waitMs = d.delayMs;
    setState(NtripState::Waiting);
}

static void fail(bool hard, const char *reason)
{
    closeClient();
    event("source %d %s: %s", sel.active() + 1, cur().name.c_str(), reason);
    scheduleNext(sel.fail(hard, millis()));
}

void ntripSetup()
{
    bool valid[CFG_MAX_SOURCES];
    for (int i = 0; i < CFG_MAX_SOURCES; i++)
        valid[i] = cfg.src[i].valid();
    sel.setFailover(cfg.failover);
    sel.setFailbackSec(cfg.failbackSec);
    sel.setRetryMs(cfg.retryMs);
    sel.setBackoff(cfg.backoffSec, 5);
    sel.setSources(CFG_MAX_SOURCES, valid);
    if (cfg.mode == "has")
    {
        setState(NtripState::Off);
        event("Galileo HAS mode: NTRIP corrections off");
        return;
    }
    setState(sel.validCount() ? NtripState::NoNetwork : NtripState::Off);
    if (state == NtripState::Off)
        event("no correction source configured");
    else
        event("%d source(s) configured, starting with %d %s", sel.validCount(), sel.active() + 1,
              cur().name.c_str());
}

static bool startConnect()
{
    const NtripSource &s = cur();
    if (s.tls)
    {
        tlsClient.setInsecure();
        client = &tlsClient;
    }
    else
        client = &plainClient;
    if (!client->connect(s.host.c_str(), s.port))
        return false;
    char req[600];
    char agent[32];
    snprintf(agent, sizeof(agent), "PostcardRTK/%s", POSTCARDRTK_VERSION);
    size_t n = ntripBuildRequest(req, sizeof(req), s.host.c_str(), s.port, s.mount.c_str(), s.user.c_str(),
                                 s.pass.c_str(), s.v2, agent);
    if (!n)
        return false;
    client->write((const uint8_t *)req, n);
    reply.reset();
    chunks.reset();
    return true;
}

// Position to send to the caster: the receiver's GGA when it has a fix, otherwise the bench position
static size_t ggaForCaster(uint32_t now, char *out, size_t outSize)
{
    SharedGga g;
    if (getLatestGga(g) && g.info.quality > 0 && (now - g.atMs) < 5000 && g.len < outSize)
    {
        memcpy(out, g.sentence, g.len);
        return g.len;
    }
    if (cfg.benchLat == 0 && cfg.benchLon == 0)
        return 0;
    time_t t = time(nullptr);
    uint32_t sod = (t > 1600000000) ? (uint32_t)(t % 86400) : 43200; // NTP time, else noon
    return nmeaMakeGga(cfg.benchLat, cfg.benchLon, cfg.benchAlt, sod, out, outSize);
}

static void sendGga(uint32_t now)
{
    if (!cur().gga || (now - lastGgaSent) < cfg.ggaSec * 1000UL)
        return;
    char gga[NMEA_MAX + 3];
    size_t n = ggaForCaster(now, gga, sizeof(gga));
    if (!n)
        return;
    client->write((const uint8_t *)gga, n);
    lastGgaSent = now;
    if (ggaSentCount++ == 0 || streamBytes == 0)
        event("GGA sent: %.60s", gga); // log until the caster answers with data
    size_t m = n;
    while (m && (gga[m - 1] == '\r' || gga[m - 1] == '\n'))
        m--;
    if (m >= sizeof(lastGgaText))
        m = sizeof(lastGgaText) - 1;
    memcpy(lastGgaText, gga, m);
    lastGgaText[m] = 0;
}

static void pushStream(const uint8_t *data, size_t n, uint32_t now)
{
    for (size_t i = 0; i < n; i++)
    {
        if (framer.feed(data[i]))
        {
            if (rtcmToRx.write(framer.frame(), framer.frameLength()))
                stats.rtcmFramesToRx++;
            lastFrameMs = now;
        }
    }
}

void ntripLoop(bool networkUp, bool allowed)
{
    uint32_t now = millis();

    if (state == NtripState::Off || state == NtripState::Stopped)
        return;

    // The method ladder holds RTK back (it did not perform better than Galileo HAS / SBAS here)
    if (!allowed)
    {
        if (state != NtripState::Paused)
        {
            closeClient();
            event("RTK paused by the method ladder, receiver falls back to Galileo HAS / SBAS");
            setState(NtripState::Paused);
        }
        return;
    }

    // Losing the network is not the source's fault: keep the source, wait for the network
    if (!networkUp)
    {
        if (state != NtripState::NoNetwork)
        {
            closeClient();
            event("network down, waiting");
            setState(NtripState::NoNetwork);
        }
        return;
    }

    switch (state)
    {
    case NtripState::Paused:
        event("RTK resumed by the method ladder");
        // fall through
    case NtripState::NoNetwork:
        waitMs = 1000;
        setState(NtripState::Waiting);
        break;

    case NtripState::Waiting:
        if (now - stateAt >= waitMs)
            setState(cur().gga ? NtripState::WaitFix : NtripState::Connecting);
        break;

    case NtripState::WaitFix: {
        char tmp[NMEA_MAX + 3];
        bool havePos = ggaForCaster(now, tmp, sizeof(tmp)) > 0;
        if (havePos || now - stateAt > WAIT_FIX_MAX_MS)
        {
            if (!havePos)
                event("no position yet, connecting without GGA");
            setState(NtripState::Connecting);
        }
        break;
    }

    case NtripState::Connecting:
        // connect() blocks for up to the lwIP timeout; acceptable on core 0
        if (startConnect())
        {
            lastGgaSent = 0;
            setState(NtripState::Reply);
        }
        else
            fail(false, "cannot connect");
        break;

    case NtripState::Reply: {
        if (!client->connected() && !client->available())
        {
            fail(false, "closed by caster");
            break;
        }
        uint8_t buf[512];
        int n = client->available();
        if (n > 0)
        {
            n = client->read(buf, n > (int)sizeof(buf) ? sizeof(buf) : n);
            size_t off = 0;
            NtripReply r = reply.feed(buf, n, &off);
            switch (r)
            {
            case NtripReply::Incomplete:
                break;
            case NtripReply::Ok: {
                event("source %d %s streaming (%s%s)", sel.active() + 1, cur().name.c_str(), reply.statusLine(),
                      reply.chunked() ? ", chunked" : "");
                streamBytes = 0;
                sel.connected(now);
                streamStart = lastFrameMs = now;
                ageBadSince = 0;
                setState(NtripState::Streaming);
                lastGgaSent = 0;
                sendGga(now);
                if (off < (size_t)n)
                {
                    size_t m = n - off;
                    if (reply.chunked())
                        m = chunks.decode(buf + off, m, buf + off);
                    bytesTotal += m;
                    streamBytes += m;
                    pushStream(buf + off, m, now);
                }
                break;
            }
            case NtripReply::Sourcetable:
                fail(true, "unknown mount point");
                break;
            case NtripReply::Unauthorized:
                fail(true, "login rejected (401)");
                break;
            case NtripReply::HardError:
                fail(true, reply.statusLine());
                break;
            case NtripReply::SoftError:
                fail(false, reply.statusLine()[0] ? reply.statusLine() : "bad reply");
                break;
            }
        }
        else if (now - stateAt > REPLY_TIMEOUT_MS)
            fail(false, "no reply");
        break;
    }

    case NtripState::Streaming: {
        uint8_t buf[1024];
        int n = client->available();
        if (n > 0)
        {
            n = client->read(buf, n > (int)sizeof(buf) ? sizeof(buf) : n);
            if (n > 0)
            {
                size_t m = n;
                if (reply.chunked())
                {
                    m = chunks.decode(buf, n, buf);
                    if (chunks.error())
                    {
                        fail(false, "bad chunked stream");
                        break;
                    }
                }
                bytesTotal += m;
                if (streamBytes == 0 && m)
                    event("first data from caster: %u bytes, first byte 0x%02X", (unsigned)m, buf[0]);
                streamBytes += m;
                pushStream(buf, m, now);
            }
        }
        else if (!client->connected())
        {
            fail(false, "connection broken");
            break;
        }
        sendGga(now);

        if (now - lastFrameMs > cfg.dataTimeoutSec * 1000UL)
        {
            fail(false, "no RTCM data");
            break;
        }

        // Corrections arriving but the receiver reports a stale correction age
        SharedGga g;
        if (cfg.maxAgeSec && now - streamStart > AGE_GRACE_MS && getLatestGga(g) && g.info.age >= 0 &&
            (g.info.quality == 4 || g.info.quality == 5))
        {
            if (g.info.age > cfg.maxAgeSec)
            {
                if (!ageBadSince)
                    ageBadSince = now;
                else if (now - ageBadSince > AGE_BAD_HOLD_MS)
                {
                    fail(false, "correction age too high");
                    break;
                }
            }
            else
                ageBadSince = 0;
        }

        if (sel.failbackDue(now))
        {
            closeClient();
            sel.reset(false);
            event("returning to source %d %s", sel.active() + 1, cur().name.c_str());
            waitMs = cfg.retryMs;
            setState(NtripState::Waiting);
        }
        break;
    }

    default:
        break;
    }
}

void ntripStop()
{
    closeClient();
    event("stopped for shutdown");
    setState(NtripState::Off);
}

// Correction mode changed at run time: drop any stream and start again under the new mode
void ntripRestart()
{
    closeClient();
    ntripSetup();
}

bool ntripStreaming() { return state == NtripState::Streaming; }

void ntripStatus(NtripStatus &s)
{
    uint32_t now = millis();
    s.state = state;
    s.source = sel.active();
    s.bytes = bytesTotal;
    s.frames = framer.frames();
    s.crcErrors = framer.crcErrors();
    s.switches = sel.switches();
    s.streamSec = (state == NtripState::Streaming) ? (now - streamStart) / 1000 : 0;
    s.lastFrameAgoMs = lastFrameMs ? now - lastFrameMs : 0xFFFFFFFF;
    s.stationId = framer.stationId();
    s.backoffLevel = sel.backoffLevel();
    strncpy(s.lastEvent, lastEvent, sizeof(s.lastEvent));
    s.streamBytes = streamBytes;
    s.ggaSent = ggaSentCount;
    strncpy(s.lastGga, lastGgaText, sizeof(s.lastGga));
}
