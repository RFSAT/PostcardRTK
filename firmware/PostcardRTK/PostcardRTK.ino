// PostcardRTK - headless RTK GNSS rover firmware for the SparkFun RTK Postcard (ESP32 + Quectel LG290P),
// running on its own from USB power, without the Portability Shield. RFSAT Ltd.
//
// The LG290P computes the solution. The ESP32 picks the method (ladder.h): RTK with corrections over Wi-Fi
// (NTRIP, ordered sources with failover) or from a phone app over Bluetooth; otherwise the receiver's own
// Galileo HAS, SBAS (EGNOS) or GNSS only; and it checks regularly whether a better method has become
// available. NMEA (plus a $PRFSM status sentence) goes to Bluetooth SPP / BLE and TCP clients.
//
// Tasks: Arduino loop = Wi-Fi, NTRIP, Bluetooth, web page, console. Receiver task (higher priority) =
// LG290P UART, NMEA parsing, method ladder; it never waits on the network.
//
// Board: ESP32 Dev Module, 16 MB flash, PSRAM on, partitions as SparkFun RTK Everywhere (tools/build.sh).

#include "config.h"
#include "console.h"
#include "diag.h"
#include "eventlog.h"
#include "history.h"
#include "net.h"
#include "ntrip.h"
#include "outputs.h"
#include "receiver.h"
#include "shared.h"
#include "util.h"
#include "web.h"
#include <LittleFS.h>
#include <WiFi.h>
#include <esp_sleep.h>
#include <esp_system.h>
#include <esp_task_wdt.h>
#include <stdarg.h>

#ifndef POSTCARDRTK_VERSION
#define POSTCARDRTK_VERSION "dev"
#endif

// Read by the TLS library of the ESP32 core as patched by SparkFun for RTK Everywhere (lets TLS buffers go to
// PSRAM); harmless with the unpatched core, where nothing reads it
bool RTK_CONFIG_MBEDTLS_EXTERNAL_MEM_ALLOC = false;

static uint32_t bootCmdsAt = 0, modeCheckUntil = 0;
static uint32_t modeReplySeq = 0;

void dbg(const char *fmt, ...)
{
    char buf[192];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    Serial.print(buf);
}

// Main loop heartbeat: the receiver task feeds the task watchdog only while the main loop is alive,
// so a hang anywhere restarts the board instead of leaving it dead
std::atomic<uint32_t> core0Beat{0};
std::atomic<bool> wdtArmed{false};
static const uint32_t WDT_MS = 10000;
static const uint32_t CORE0_STALL_MS = 60000; // longest acceptable blocking call in the main loop (TLS connect)

static void stopEverything(const char *why)
{
    diagC0 = DIAG_CLEAN;
    eventAdd(why);
    eventsFlush();
    historyFlush(); // keep the last minutes of history
    ntripStop();
    outputsStop();
    core1Halt = true; // the receiver task stops using the UART
    delay(100);
    wdtArmed = false;
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
}

void safeReboot()
{
    dbg("[boot] restarting\r\n");
    stopEverything("restart requested");
    esp_restart();
    while (true)
        ;
}

void safeShutdown()
{
    dbg("[boot] shutting down\r\n");
    stopEverything("shut down from the page");
    digitalWrite(PIN_GNSS_RESET, LOW); // hold the receiver in reset (lower current)
    dbg("[boot] halted, safe to unplug; press reset or power it up again to start\r\n");
    Serial.flush();
    esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
    esp_deep_sleep_start(); // no wake-up source: stays off until reset or power cycle
    while (true)
        ;
}

static const char *resetReasonName(esp_reset_reason_t r)
{
    switch (r)
    {
    case ESP_RST_POWERON: return "power-on";
    case ESP_RST_EXT: return "reset pin";
    case ESP_RST_SW: return "software restart";
    case ESP_RST_PANIC: return "crash";
    case ESP_RST_INT_WDT:
    case ESP_RST_TASK_WDT:
    case ESP_RST_WDT: return "watchdog (recovered from a hang)";
    case ESP_RST_DEEPSLEEP: return "start after shut down";
    case ESP_RST_BROWNOUT: return "brown-out (supply dipped)";
    default: return "unknown";
    }
}
const char *bootReason = "unknown";

static void receiverTask(void *)
{
    receiverSetup1();
    esp_task_wdt_add(nullptr);
    for (;;)
    {
        diagC1 = 1;
        receiverLoop1();
        diagC1 = 0;
        if (!wdtArmed || millis() - core0Beat.load() < CORE0_STALL_MS)
            esp_task_wdt_reset();
        vTaskDelay(1); // 1 ms: at 460800 baud about 46 bytes arrive per tick; the UART buffer holds 16 KB
    }
}

void setup()
{
    Serial.begin(115200);
    RTK_CONFIG_MBEDTLS_EXTERNAL_MEM_ALLOC = psramFound();
    const esp_reset_reason_t rr = esp_reset_reason();
    bootReason = resetReasonName(rr);
    diagBoot((int)rr);
    dbg("\r\n[boot] PostcardRTK %s, last reset: %s\r\n[boot] previous run: %s\r\n", POSTCARDRTK_VERSION, bootReason,
        diagText().c_str());
    diagC0 = 101;
    receiverInitShared();
    diagC0 = 102;
    if (!LittleFS.begin(true)) // formats the partition the first time
        dbg("[boot] flash file system not available\r\n");
    diagC0 = 103;
    configLoad();
    ladderConfigure();
    diagC0 = 104;
    {
        RefPoint r; // the reference point is part of the settings
        r.set = cfg.refSet;
        r.lat = cfg.refLat;
        r.lon = cfg.refLon;
        r.alt = cfg.refAlt;
        setRefPoint(r);
    }
    diagC0 = 105;
    historySetup();
    eventsSetup();
    {
        char m[200];
        snprintf(m, sizeof(m), "start-up: firmware %s, last reset: %s; previous run %s", POSTCARDRTK_VERSION, bootReason,
                 diagText().c_str());
        eventAdd(m);
    }
    diagC0 = 106;
    configReady = true; // the receiver task may start the UART now
    core0Beat = millis();
    esp_task_wdt_config_t wdc = {};
    wdc.timeout_ms = WDT_MS;
    wdc.idle_core_mask = 0;
    wdc.trigger_panic = true;
    esp_task_wdt_reconfigure(&wdc);
    xTaskCreatePinnedToCore(receiverTask, "receiver", 8192, nullptr, 3, nullptr, 1);
    wdtArmed = true;

    diagC0 = 108;
    outputsSetup();
    dbg("[boot] Bluetooth %s as %s\r\n", cfg.bt.c_str(), outputsBtName().c_str());
    diagC0 = 109;
    netSetup();
    diagC0 = 110;
    webSetup();
    diagC0 = 111;
    ntripSetup();
    diagC0 = 112;
    bootCmdsAt = millis() + 3000; // let the receiver boot first
    dbg("[boot] ready. Type help for commands.\r\n");
}

void consolePrintStatus()
{
    NtripStatus s;
    ntripStatus(s);
    SharedGga g;
    bool gga = getLatestGga(g);
    static Ladder l;
    getLadder(l);
    OutputStatus o;
    outputsStatus(o);
    dbg("[postcardrtk %s] fix=%s sats=%d | ladder=%s | wifi=%s %s %s | ntrip=%s src=%d | bt spp=%d ble=%d | uart rx=%lu B\r\n",
        POSTCARDRTK_VERSION, gga ? methodName(classifyMethod(g.info, nullptr, false)) : "no data", gga ? g.info.sats : 0,
        ladderStateName(l.state()), netStateName(), netSsid().c_str(), netIp().c_str(), ntripStateName(s.state),
        s.source + 1, o.sppConnected, o.bleConnected, (unsigned long)stats.uartRxBytes.load());
}

// Status line on USB every 30 s
static void logStatus()
{
    static uint32_t last = 0;
    if (millis() - last < 30000)
        return;
    last = millis();
    consolePrintStatus();
}

static void sendRx(const char *c)
{
    char out[140];
    size_t n = nmeaFinish(c, out, sizeof(out));
    if (n)
        cmdToRx.write((const uint8_t *)out, n);
}

// Galileo HAS (PPP, E6) in the receiver: on unless switched off in the settings.
// WGS84, 120 s timeout, converged at 0.10 m horizontal / 0.15 m vertical (Quectel LG290P PPP application note).
// Not saved in the receiver: sent at every start-up and on every mode change.
void sendHasCommand() { sendRx((cfg.mode == "has" || cfg.galileoHas) ? "$PQTMCFGPPP,W,2,1,120,0.10,0.15" : "$PQTMCFGPPP,W,0"); }

static void sendBootCommands()
{
    // The messages this firmware needs on its UART (not saved in the receiver)
    static const char *const cmds[] = {"$PQTMCFGMSGRATE,W,GGA,1",        "$PQTMCFGMSGRATE,W,RMC,1",
                                       "$PQTMCFGMSGRATE,W,GSA,1",        "$PQTMCFGMSGRATE,W,GSV,1",
                                       "$PQTMCFGMSGRATE,W,GST,1",        "$PQTMCFGMSGRATE,W,PQTMEPE,1,2",
                                       "$PQTMCFGMSGRATE,W,PQTMPPPNAV,1,1", "$PQTMCFGRCVRMODE,R"};
    sendHasCommand();
    for (const char *c : cmds)
        sendRx(c);
    for (int i = 0; i < CFG_MAX_BOOTCMDS; i++)
        if (cfg.bootCmd[i].length())
            sendRx(cfg.bootCmd[i].c_str());
    modeReplySeq = rxReplySeq;
    modeCheckUntil = millis() + 10000;
}

// The receiver must be a rover. A Postcard used with RTK Everywhere may have been left in base mode
// (it then sends RTCM instead of using it): switch it back, save, restart the receiver.
static void checkReceiverMode()
{
    if (!modeCheckUntil)
        return;
    if ((int32_t)(millis() - modeCheckUntil) > 0)
    {
        modeCheckUntil = 0;
        return;
    }
    char r[NMEA_MAX + 1] = "";
    mutex_enter_blocking(&typesMutex);
    for (uint32_t k = modeReplySeq; k < rxReplySeq; k++)
        if (strncmp(rxReplies[k % RX_REPLIES], "$PQTMCFGRCVRMODE,OK,", 20) == 0)
            strlcpy(r, rxReplies[k % RX_REPLIES], sizeof(r));
    mutex_exit(&typesMutex);
    if (!r[0])
        return;
    modeCheckUntil = 0;
    int mode = atoi(r + 20);
    if (mode == 2)
    {
        eventAdd("receiver was in base mode: switched to rover, saved, receiver restarted");
        dbg("[rx] receiver in base mode, switching to rover\r\n");
        sendRx("$PQTMCFGRCVRMODE,W,1");
        sendRx("$PQTMSAVEPAR");
        sendRx("$PQTMSRR");
        bootCmdsAt = millis() + 5000; // the message settings are lost with the restart: send them again
    }
}

void loop()
{
    core0Beat = millis();
    diagC0 = 1;
    netLoop();
    diagC0 = 2;
    const bool up = netUp();
    outputsNetworkUp(up);
    diagC0 = 3;
    webLoop();
    diagC0 = 4;
    NtripStatus ns;
    ntripStatus(ns);
    const bool casterConfigured = ns.state != NtripState::Off && ns.state != NtripState::Stopped;
    ladderSourceAvailable = (up && casterConfigured) || outputsInboundRtcmActive();
    const bool allow = ladderAllowRtk.load();
    ntripLoop(up, allow);
    diagC0 = 5;
    outputsLoop(ntripStreaming(), allow);
    diagC0 = 6;
    consoleLoop();
    logStatus();
    {
        char reason[160];
        LadderState st;
        if (ladderTakeChange(reason, sizeof(reason), st))
        {
            char m[200];
            snprintf(m, sizeof(m), "method ladder: %s - %s", ladderStateName(st), reason);
            eventAdd(m);
            dbg("[ladder] %s - %s\r\n", ladderStateName(st), reason);
        }
    }
    diagC0 = 8;
    historyLoop();
    eventsLoop();
    diagC0 = 9;

    if (bootCmdsAt && (int32_t)(millis() - bootCmdsAt) > 0)
    {
        bootCmdsAt = 0;
        sendBootCommands();
    }
    checkReceiverMode();
    delay(1);
}
