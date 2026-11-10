// web.cpp - core 0: configuration page and JSON API (HTTP Basic login "admin" / adminPass)
#include "web.h"
#include "board.h"
#include "config.h"
#include "history.h"
#include "eventlog.h"
#include "diag.h"
#include "net.h"
#include "ntrip.h"
#include "outputs.h"
#include "receiver.h"
#include "shared.h"
#include "util.h"
#include "web_page.h"
#include <ArduinoJson.h>
#include <WebServer.h>
#include <WiFi.h>

#ifndef POSTCARDRTK_VERSION
#define POSTCARDRTK_VERSION "0.1.0"
#endif

static WebServer server(80);
// One copy of the method tracker for the page handlers (they run one at a time), on the heap
static MethodTracker &webTracker()
{
    static MethodTracker *m = new MethodTracker();
    return *m;
}
static uint32_t rebootAt = 0, shutdownAt = 0;

static bool auth()
{
    if (server.authenticate("admin", cfg.adminPass.c_str()))
        return true;
    server.requestAuthentication(BASIC_AUTH, "PostcardRTK");
    return false;
}

static const char *qualityName(int q)
{
    switch (q)
    {
    case 0: return "no fix";
    case 1: return "GNSS";
    case 2: return "DGNSS";
    case 4: return "RTK fixed";
    case 5: return "RTK float";
    case 6: return "dead reckoning";
    default: return "other";
    }
}

static void handleStatus()
{
    if (!auth())
        return;
    uint32_t now = millis();
    JsonDocument d;
    d["version"] = POSTCARDRTK_VERSION;
    d["board"] = BOARD_NAME;
    d["btClassic"] = BOARD_BT_CLASSIC != 0;
    d["uptimeSec"] = now / 1000;
    extern const char *bootReason;
    d["lastReset"] = bootReason;
    d["eventSeq"] = eventsSeq();
    d["previousRun"] = diagText();
    d["previousRunProblem"] = diagProblem();
    d["device"] = cfg.deviceName;
    d["privateDefaults"] = configHasPrivateDefaults();
    d["mode"] = cfg.mode;
    d["galileoHas"] = cfg.galileoHas;
    d["bluetoothName"] = outputsBtName();
    {
        static Ladder l; // copied under the receiver's lock
        getLadder(l);
        JsonObject lo = d["ladder"].to<JsonObject>();
        lo["state"] = ladderStateName(l.state());
        lo["code"] = ladderStateCode(l.state());
        lo["reason"] = l.reason();
        lo["pauses"] = l.pauseCount();
        if (l.state() == LS_RTK_PAUSED)
            lo["retryInSec"] = (l.pausedUntilMs() - now) / 1000;
        if (l.altBest() >= 0)
            lo["fallbackBestM"] = l.altBest();
    }
    d["receiverBaud"] = receiverBaud.load();

    JsonObject w = d["wifi"].to<JsonObject>();
    w["mode"] = netApMode() ? "setup access point" : (netUp() ? "connected" : netStateName());
    w["ssid"] = netSsid();
    w["ip"] = netIp();
    w["rssi"] = netRssi();

    SharedGga g;
    JsonObject n = d["gnss"].to<JsonObject>();
    if (getLatestGga(g))
    {
        n["quality"] = g.info.quality;
        n["qualityName"] = qualityName(g.info.quality);
        n["sats"] = g.info.sats;
        n["hdop"] = g.info.hdop;
        n["lat"] = serialized(String(g.info.lat, 9));
        n["lon"] = serialized(String(g.info.lon, 9));
        n["alt"] = g.info.alt;
        n["age"] = g.info.age;
        n["station"] = g.info.station;
        n["utc"] = g.info.utc;
        n["ggaAgeMs"] = now - g.atMs;
    }
    else
        n["qualityName"] = "no data from receiver";

    // Accuracy estimates (only those that arrived in the last 5 s)
    SharedAccuracy a;
    getAccuracy(a);
    if (a.gstAtMs && now - a.gstAtMs < 5000)
    {
        n["gstH"] = sqrtf(a.gstLat * a.gstLat + a.gstLon * a.gstLon);
        n["gstV"] = a.gstAlt;
    }
    if (a.epeAtMs && now - a.epeAtMs < 5000)
    {
        n["epeH"] = a.epe2d;
        n["epeV"] = fabsf(a.epeDown);
    }
    if (a.pppAtMs && now - a.pppAtMs < 5000)
    {
        JsonObject p = d["ppp"].to<JsonObject>();
        p["sol"] = a.ppp.sol;
        if (a.ppp.latStd >= 0 && a.ppp.lonStd >= 0)
            p["h"] = sqrtf(a.ppp.latStd * a.ppp.latStd + a.ppp.lonStd * a.ppp.lonStd);
        if (a.ppp.altStd >= 0)
            p["v"] = a.ppp.altStd;
        p["diffId"] = a.ppp.diffId;
        p["age"] = a.ppp.diffAge;
        p["sats"] = a.ppp.satUsed;
    }

    // Methods over time (the receiver computes one at a time), their corrections, reference point, movement
    {
        MethodTracker &mt = webTracker(); // large: kept off the stack
        RefPoint rp;
        getMethods(mt, rp);
        JsonObject m = d["methods"].to<JsonObject>();
        m["current"] = methodName(mt.current());
        m["currentCode"] = (int)mt.current();
        m["currentForSec"] = (now - mt.currentSinceMs()) / 1000;
        if (mt.speed() >= 0) m["speed"] = mt.speed();
        m["moving"] = mt.moving();
        int curGrp = methodGroup(mt.current());
        JsonArray ga = m["groups"].to<JsonArray>();
        for (int i = 0; i < G_COUNT; i++)
        {
            const MethodStat &st = mt.stat(i);
            JsonObject o = ga.add<JsonObject>();
            o["name"] = groupName((MethodGroup)i);
            o["seen"] = st.seen;
            if (!st.seen)
                continue;
            o["running"] = i == curGrp && now - st.lastMs < 5000;
            o["last"] = methodName(st.last);
            o["runSec"] = (st.lastMs - st.startMs) / 1000;
            o["endedAgoSec"] = (now - st.lastMs) / 1000;
            if (st.estH >= 0) o["estH"] = st.estH;
            if (st.estV >= 0) o["estV"] = st.estV;
            if (st.errH >= 0) o["errH"] = st.errH;
            if (st.errV >= 0) o["errV"] = st.errV;
            if (st.bestEstH >= 0) o["bestEstH"] = st.bestEstH;
            JsonObject c = o["corr"].to<JsonObject>();
            c["live"] = st.corr.live;
            if (st.corr.age >= 0) c["age"] = st.corr.age;
            if (st.corr.station >= 0) c["station"] = st.corr.station;
        }
        JsonObject r = m["ref"].to<JsonObject>();
        r["set"] = rp.set;
        if (rp.set)
        {
            r["lat"] = serialized(String(rp.lat, 9));
            r["lon"] = serialized(String(rp.lon, 9));
            r["alt"] = rp.alt;
        }
    }

    NtripStatus s;
    ntripStatus(s);
    JsonObject t = d["ntrip"].to<JsonObject>();
    t["state"] = ntripStateName(s.state);
    t["source"] = s.source + 1;
    t["sourceName"] = cfg.src[s.source].name;
    t["mount"] = cfg.src[s.source].mount;
    t["host"] = cfg.src[s.source].host;
    t["bytes"] = s.bytes;
    t["streamBytes"] = s.streamBytes;
    t["ggaSent"] = s.ggaSent;
    t["lastGga"] = s.lastGga;
    t["frames"] = s.frames;
    t["crcErrors"] = s.crcErrors;
    t["switches"] = s.switches;
    t["streamSec"] = s.streamSec;
    if (s.lastFrameAgoMs != 0xFFFFFFFF)
        t["lastFrameAgoMs"] = s.lastFrameAgoMs;
    t["stationId"] = s.stationId;
    t["backoffLevel"] = s.backoffLevel;
    t["lastEvent"] = s.lastEvent;

    OutputStatus o;
    outputsStatus(o);
    JsonObject out = d["outputs"].to<JsonObject>();
    out["bt"] = cfg.bt;
    out["spp"] = o.sppConnected;
    out["ble"] = o.bleConnected;
    out["tcpClients"] = o.tcpClients;
    out["tcpPort"] = cfg.tcpPort;
    out["sentBytes"] = o.sentBytes;
    out["inboundFrames"] = o.inboundFrames;

    JsonObject u = d["uart"].to<JsonObject>();
    u["baud"] = cfg.uartBaud;
    u["rxBytes"] = stats.uartRxBytes.load();
    u["txBytes"] = stats.uartTxBytes.load();
    u["nmeaGood"] = stats.nmeaGood.load();
    u["nmeaBad"] = stats.nmeaBad.load();
    u["rtcmFrames"] = stats.rtcmFramesToRx.load();
    u["drops"] = rtcmToRx.dropped() + nmeaToNet.dropped();

    String js;
    serializeJson(d, js);
    server.sendHeader("Cache-Control", "no-store");
    server.send(200, "application/json", js);
}

static void handleGetConfig()
{
    if (!auth())
        return;
    bool secrets = server.hasArg("secrets") && server.arg("secrets") == "1";
    if (secrets)
        server.sendHeader("Content-Disposition", "attachment; filename=\"postcardrtk-config.json\"");
    server.sendHeader("Cache-Control", "no-store");
    server.send(200, "application/json", configToJson(secrets));
}

static void handlePostConfig()
{
    if (!auth())
        return;
    String err;
    if (!configFromJson(server.arg("plain"), err))
    {
        server.send(400, "text/plain", err);
        return;
    }
    if (!configSave())
    {
        server.send(500, "text/plain", "Could not write the settings to flash");
        return;
    }
    server.send(200, "text/plain", "Saved. Restarting in 2 s.");
    rebootAt = millis() + 2000;
}

static void handleCmd()
{
    if (!auth())
        return;
    String c = server.arg("plain");
    c.trim();
    if (!c.startsWith("$") || c.length() > 120)
    {
        server.send(400, "text/plain", "A receiver command starts with $ and is at most 120 characters");
        return;
    }
    char out[130];
    size_t n = nmeaFinish(c.c_str(), out, sizeof(out));
    if (!n || !cmdToRx.write((const uint8_t *)out, n))
    {
        server.send(503, "text/plain", "Command queue full, try again");
        return;
    }
    server.send(200, "text/plain", String("Sent ") + out);
}

static void handleLog()
{
    if (!auth())
        return;
    String s;
    eventsText(s);
    server.sendHeader("Cache-Control", "no-store");
    server.send(200, "text/plain", s);
}

static void handleScan()
{
    if (!auth())
        return;
    int n = WiFi.scanNetworks();
    JsonDocument d;
    JsonArray a = d.to<JsonArray>();
    for (int i = 0; i < n && i < 20; i++)
    {
        JsonObject o = a.add<JsonObject>();
        o["ssid"] = WiFi.SSID(i);
        o["rssi"] = WiFi.RSSI(i);
    }
    String js;
    serializeJson(d, js);
    server.send(200, "application/json", js);
}

void webSetup()
{
    server.on("/", HTTP_GET, []() {
        if (!auth())
            return;
        server.send_P(200, "text/html", INDEX_HTML);
    });
    server.on("/api/status", HTTP_GET, handleStatus);
    server.on("/api/config", HTTP_GET, handleGetConfig);
    server.on("/api/config", HTTP_POST, handlePostConfig);
    server.on("/api/cmd", HTTP_POST, handleCmd);
    server.on("/api/log", HTTP_GET, handleLog);
    server.on("/api/scan", HTTP_GET, handleScan);
    server.on("/api/reboot", HTTP_POST, []() {
        if (!auth())
            return;
        server.send(200, "text/plain", "Restarting");
        rebootAt = millis() + 1000;
    });
    // Switch between RTK (NTRIP) and Galileo HAS without a restart; saved, so it survives power cycles
    server.on("/api/mode", HTTP_POST, []() {
        if (!auth())
            return;
        String m = server.arg("plain");
        m.trim();
        if (m != "rtk" && m != "has")
        {
            server.send(400, "text/plain", "Mode must be rtk or has");
            return;
        }
        cfg.mode = m;
        if (!configSave())
        {
            server.send(500, "text/plain", "Could not write the settings to flash");
            return;
        }
        extern void sendHasCommand();
        sendHasCommand();
        ntripRestart();
        server.send(200, "text/plain", m == "has" ? "Galileo HAS mode: NTRIP off. Convergence takes several minutes." : "RTK mode: NTRIP corrections on.");
    });
    // Reference point: "set" = current position, only from an RTK fixed solution; "force" = any fix; "clear"
    server.on("/api/ref", HTTP_POST, []() {
        if (!auth())
            return;
        String a = server.arg("plain");
        a.trim();
        RefPoint r;
        if (a == "set" || a == "force")
        {
            SharedGga g;
            MethodTracker &mt = webTracker();
            RefPoint old;
            getMethods(mt, old);
            if (!getLatestGga(g) || millis() - g.atMs > 3000 || g.info.quality == 0)
            {
                server.send(409, "text/plain", "No current position from the receiver");
                return;
            }
            if (a == "set" && mt.current() != M_RTK_FIXED)
            {
                server.send(409, "text/plain", "Needs an RTK fixed solution (now: " + String(methodName(mt.current())) + ")");
                return;
            }
            r.set = true;
            r.lat = g.info.lat;
            r.lon = g.info.lon;
            r.alt = g.info.alt;
        }
        else if (a != "clear")
        {
            server.send(400, "text/plain", "Use set, force or clear");
            return;
        }
        cfg.refSet = r.set;
        cfg.refLat = r.lat;
        cfg.refLon = r.lon;
        cfg.refAlt = r.alt;
        if (!configSave())
        {
            server.send(500, "text/plain", "Could not write the settings to flash");
            return;
        }
        setRefPoint(r);
        server.send(200, "text/plain", r.set ? "Reference point recorded at the current position." : "Reference point cleared.");
    });
    // Accuracy trace: last 30 min, one sample every 10 s: [uptime s, method code, estimate H, error H] (-1 = none)
    // Accuracy chart. From the flash history (so it survives restarts), on UTC time:
    // ?span=seconds (600..86400, default 1800). At most 720 points: every k-th record for long spans.
    // samples: [utc s, method code, estimate H, error H] (-1 = none), oldest first.
    // While no record has a date yet (no fix since the history began), it sends this run's uptime
    // samples instead, with "mode":"uptime".
    server.on("/api/trace", HTTP_GET, []() {
        if (!auth())
            return;
        long span = server.hasArg("span") ? server.arg("span").toInt() : 1800;
        span = span < 600 ? 600 : span > 86400 ? 86400 : span;
        struct Pt
        {
            int64_t t;
            uint8_t m;
            float e, r;
        };
        static Pt *pts = new Pt[720]; // on the heap: static RAM is scarce next to Wi-Fi and Bluetooth
        struct Ctx
        {
            int64_t now = -1, span = 1800;
            uint32_t k = 1, count = 0;
            int n = 0;
        } c;
        c.span = span;
        c.k = (uint32_t)((span / 10 + 719) / 720);
        historyBackward(
            [](const HistRec &r, void *p) -> bool {
                Ctx *x = (Ctx *)p;
                const int64_t t = histEpoch(r);
                if (t < 0)
                    return true; // restart marker or no date yet: the time gap shows it
                if (x->now < 0)
                    x->now = t;
                if (t < x->now - x->span || x->n >= 720)
                    return false;
                if (x->count++ % x->k == 0)
                    pts[x->n++] = Pt{t, r.method, r.estH, r.errH};
                return true;
            },
            &c);
        String js;
        js.reserve(c.n * 34 + 96);
        if (c.now >= 0)
        {
            js = "{\"mode\":\"utc\",\"now\":" + String((long)c.now) + ",\"span\":" + String(span) + ",\"step\":" + String(10 * c.k) +
                 ",\"samples\":[";
            for (int i = c.n - 1; i >= 0; i--) // oldest first
            {
                js += '[';
                js += String((long)pts[i].t);
                js += ',';
                js += String(pts[i].m);
                js += ',';
                js += String(pts[i].e, 3);
                js += ',';
                js += String(pts[i].r, 3);
                js += i ? "]," : "]";
            }
            js += "]}";
        }
        else
        {
            MethodTracker &mt = webTracker();
            RefPoint rp;
            getMethods(mt, rp);
            js = "{\"mode\":\"uptime\",\"now\":" + String(millis() / 1000) + ",\"span\":1800,\"step\":" + String(TRACE_STEP_MS / 1000) +
                 ",\"samples\":[";
            for (int i = 0; i < mt.traceCount(); i++)
            {
                const TraceSample &t = mt.trace(i);
                js += i ? ",[" : "[";
                js += String(t.tSec);
                js += ',';
                js += String(t.method);
                js += ',';
                js += String(t.estH, 3);
                js += ',';
                js += String(t.errH, 3);
                js += ']';
            }
            js += "]}";
        }
        server.sendHeader("Cache-Control", "no-store");
        server.send(200, "application/json", js);
    });
    // Satellites in view: position, C/N0 for every signal (band name), used in the solution
    server.on("/api/sky", HTTP_GET, []() {
        if (!auth())
            return;
        static SkyTable &t = *new SkyTable(); // on the heap
        getSky(t);
        const uint32_t now = millis();
        String js;
        js.reserve(4096);
        const int64_t utcNow = currentUtc();
        js = "{\"gsvAgeMs\":" + String(t.lastGsvMs() ? (long)(now - t.lastGsvMs()) : -1) +
             ",\"gsaAgeMs\":" + String(t.lastGsaMs() ? (long)(now - t.lastGsaMs()) : -1) + ",\"utc\":" + String((long)utcNow) +
             ",\"sats\":[";
        bool first = true;
        for (int i = 0; i < t.count(); i++)
        {
            const SkySat &x = t.at(i);
            if (!x.sys)
                continue;
            js += first ? "{\"s\":\"" : ",{\"s\":\"";
            first = false;
            js += x.sys;
            js += "\",\"p\":" + String(x.prn) + ",\"el\":" + String(x.el) + ",\"az\":" + String(x.az) +
                  ",\"a\":" + String((now - x.seenMs) / 1000) + ",\"u\":" +
                  (SkyTable::used(x, now) ? "1" : "0") + ",\"sig\":[";
            bool fs = true;
            for (const SkySig &g : x.sig)
            {
                if (!g.id)
                    continue;
                js += fs ? "[\"" : ",[\"";
                fs = false;
                js += skyBandName(x.sys, g.id);
                js += "\"," + String(g.cn0) + "," + String(skyBandOrder(x.sys, g.id)) + "]";
            }
            js += "]}";
        }
        js += "]";
        // Last known sky from the previous run, until 3 min after the first live GSV: the satellites
        // not seen yet in this run, for the page to show greyed
        const SkySnap &k = skyLastKnown();
        if (k.magic == SKY_SNAP_MAGIC && k.n && (!t.firstGsvMs() || now - t.firstGsvMs() < 180000))
        {
            js += ",\"lastKnown\":{\"utc\":" + String((long)k.utc) + ",\"sats\":[";
            bool fk = true;
            for (uint32_t i = 0; i < k.n; i++)
            {
                const SkySnapSat &q = k.s[i];
                bool inTable = false;
                for (int j = 0; j < t.count() && !inTable; j++)
                    inTable = t.at(j).sys == q.sys && t.at(j).prn == q.prn;
                if (inTable)
                    continue;
                js += fk ? "{\"s\":\"" : ",{\"s\":\"";
                fk = false;
                js += q.sys;
                js += "\",\"p\":" + String(q.prn) + ",\"el\":" + String(q.el) + ",\"az\":" + String(q.az) + ",\"u\":" + String(q.used) + "}";
            }
            js += "]}";
        }
        js += "}";
        server.sendHeader("Cache-Control", "no-store");
        server.send(200, "application/json", js);
    });
    // Positioning history kept in flash across restarts
    server.on("/api/history", HTTP_GET, []() {
        if (!auth())
            return;
        uint32_t fy, fm, ly, lm;
        historyInfo(fy, fm, ly, lm);
        JsonDocument d;
        d["records"] = historyRecords();
        d["bytes"] = historyBytes();
        d["capacityRecords"] = 2 * HIST_FILE_MAX / sizeof(HistRec);
        d["recordBytes"] = sizeof(HistRec);
        d["stepSec"] = TRACE_STEP_MS / 1000;
        d["firstYmd"] = fy;
        d["firstMs"] = fm;
        d["lastYmd"] = ly;
        d["lastMs"] = lm;
        String js;
        serializeJson(d, js);
        server.sendHeader("Cache-Control", "no-store");
        server.send(200, "application/json", js);
    });
    server.on("/api/history.csv", HTTP_GET, []() {
        if (!auth())
            return;
        historyFlush();
        uint32_t fy, fm, ly, lm;
        historyInfo(fy, fm, ly, lm);
        char name[64];
        snprintf(name, sizeof(name), "attachment; filename=\"postcardrtk-history-%lu.csv\"", (unsigned long)(ly ? ly : 0));
        server.sendHeader("Content-Disposition", name);
        server.sendHeader("Cache-Control", "no-store");
        server.setContentLength(CONTENT_LENGTH_UNKNOWN);
        server.send(200, "text/csv", "");
        struct Ctx
        {
            String chunk;
        } ctx;
        ctx.chunk.reserve(1600);
        ctx.chunk = HIST_CSV_HEADER;
        historyForEach(
            [](const HistRec &r, void *c) -> bool {
                Ctx *x = (Ctx *)c;
                char line[200];
                size_t n = histCsvLine(r, line, sizeof(line));
                x->chunk.concat(line, n);
                if (x->chunk.length() > 1400)
                {
                    extern std::atomic<uint32_t> core0Beat;
                    core0Beat = millis(); // a long download must not look like a stalled core 0
                    server.sendContent(x->chunk);
                    x->chunk = "";
                }
                return server.client().connected();
            },
            &ctx);
        if (ctx.chunk.length())
            server.sendContent(ctx.chunk);
        server.sendContent("");
    });
    server.on("/api/history/clear", HTTP_POST, []() {
        if (!auth())
            return;
        bool ok = historyClear();
        server.send(ok ? 200 : 500, "text/plain", ok ? "History cleared." : "Could not delete the history files");
    });
    server.on("/api/factory", HTTP_POST, []() {
        if (!auth())
            return;
        if (!configReset())
        {
            server.send(500, "text/plain", "Could not delete the saved settings");
            return;
        }
        server.send(200, "text/plain", "Saved settings deleted. Restarting with the built-in defaults.");
        rebootAt = millis() + 1000;
    });
    server.on("/api/shutdown", HTTP_POST, []() {
        if (!auth())
            return;
        server.send(200, "text/plain", "Shutting down. Wait until the LEDs are off (about 2 s), then unplug.");
        shutdownAt = millis() + 1000;
    });
    server.on("/api/nmea", HTTP_GET, []() {
        if (!auth())
            return;
        JsonDocument d;
        JsonArray a = d["types"].to<JsonArray>();
        mutex_enter_blocking(&typesMutex);
        for (int i = 0; i < nmeaTypeCount; i++)
        {
            JsonObject o = a.add<JsonObject>();
            o["id"] = nmeaTypes[i].id;
            o["count"] = nmeaTypes[i].count;
            o["sent"] = nmeaTypes[i].sent;
        }
        // Receiver replies, oldest first
        JsonArray r = d["replies"].to<JsonArray>();
        uint32_t first = rxReplySeq > RX_REPLIES ? rxReplySeq - RX_REPLIES : 0;
        for (uint32_t k = first; k < rxReplySeq; k++)
            r.add(rxReplies[k % RX_REPLIES]);
        d["replySeq"] = rxReplySeq;
        mutex_exit(&typesMutex);
        d["filter"] = cfg.nmeaTypes;
        d["text"] = outputsMonitor();
        String js;
        serializeJson(d, js);
        server.sendHeader("Cache-Control", "no-store");
        server.send(200, "application/json", js);
    });
    server.onNotFound([]() { server.send(404, "text/plain", "Not found"); });
    server.begin();
}

void webLoop()
{
    server.handleClient();
    if (shutdownAt && (int32_t)(millis() - shutdownAt) > 0)
        safeShutdown();
    if (rebootAt && (int32_t)(millis() - rebootAt) > 0)
        safeReboot();
}
