// outputs.cpp - main loop: NMEA to Bluetooth (classic SPP for Android, BLE for iPhone and PocketGNSS 1.13) and TCP
// clients, inbound RTCM from apps that run their own NTRIP client (e.g. over the phone's mobile data).
// On the ESP32-C6 (V2) only BLE exists: the Bluetooth setting spp / both is served as BLE.
#include "outputs.h"
#include "board.h"
#include "config.h"
#include "rtcm.h"
#include "shared.h"
#include "util.h"
#include <BleSerial.h>
#include <WiFi.h>
#if BOARD_BT_CLASSIC
#include <BluetoothSerial.h>
#else
// No Bluetooth classic on this chip: an SPP that never starts, so the code below stays the same
struct NoSpp
{
    void setTimeout(int) {}
    bool begin(const String &, bool) { return false; }
    void end() {}
    bool hasClient() { return false; }
    size_t write(const uint8_t *, size_t) { return 0; }
    int available() { return 0; }
    size_t readBytes(uint8_t *, size_t) { return 0; }
};
#endif

static const int MAX_TCP = 3;
static bool useSpp = false, useBle = false;
#if BOARD_BT_CLASSIC
static BluetoothSerial spp;
#else
static NoSpp spp;
#endif
static BleSerial ble;
static WiFiServer *tcpServer = nullptr;
static WiFiClient tcpClients[MAX_TCP];
static RtcmFramer inFramer;
static uint32_t sentBytes = 0, inboundFrames = 0, lastInboundFrameMs = 0;
static String btName;

// Tail of what was sent, for the web NMEA monitor
static const size_t MON_SIZE = 3072;
static char monBuf[MON_SIZE];
static size_t monHead = 0, monFill = 0;

static void monitorAdd(const uint8_t *p, size_t n)
{
    for (size_t i = 0; i < n; i++)
    {
        monBuf[monHead] = (char)p[i];
        monHead = (monHead + 1) % MON_SIZE;
        if (monFill < MON_SIZE)
            monFill++;
    }
}

String outputsMonitor()
{
    String s;
    s.reserve(monFill + 1);
    size_t start = (monHead + MON_SIZE - monFill) % MON_SIZE;
    for (size_t i = 0; i < monFill; i++)
        s += monBuf[(start + i) % MON_SIZE];
    int nl = s.indexOf('\n'); // drop the first, possibly cut, line
    if (monFill == MON_SIZE && nl >= 0)
        s.remove(0, nl + 1);
    return s;
}

String outputsBtName() { return btName; }

void outputsSetup()
{
#if BOARD_BT_CLASSIC
    useSpp = (cfg.bt == "spp" || cfg.bt == "both");
    useBle = (cfg.bt == "ble" || cfg.bt == "both");
#else
    useSpp = false;
    useBle = (cfg.bt != "off");
#endif

    // Bluetooth name: device name + last 4 hex digits of the Wi-Fi MAC, e.g. "PostcardRTK-7924"
    uint8_t mac[6] = {0};
    WiFi.macAddress(mac);
    char suffix[8];
    snprintf(suffix, sizeof(suffix), "-%02X%02X", mac[4], mac[5]);
    btName = cfg.deviceName + suffix;

    // Classic SPP first, then BLE on the same (dual-mode) Bluetooth controller, as SparkFun RTK Everywhere does
    if (useSpp)
    {
        spp.setTimeout(10);
        if (!spp.begin(btName, false))
        {
            dbg("[bt] SPP did not start\r\n");
            useSpp = false;
        }
    }
    if (useBle)
        ble.begin(btName.c_str());
}

void outputsStop()
{
    for (auto &c : tcpClients)
        c.stop();
    if (useBle)
        ble.end();
    if (useSpp)
        spp.end();
}

void outputsNetworkUp(bool up)
{
    if (up && cfg.tcpPort && !tcpServer)
    {
        tcpServer = new WiFiServer(cfg.tcpPort);
        tcpServer->begin();
        tcpServer->setNoDelay(true);
    }
    else if (!up && tcpServer)
    {
        for (auto &c : tcpClients)
            c.stop();
        tcpServer->stop();
        delete tcpServer;
        tcpServer = nullptr;
    }
}

// Inbound bytes: RTCM frames are always recognised (so the method ladder knows an app offers corrections),
// and passed to the receiver only when accepted
static void inbound(const uint8_t *buf, size_t n, bool accept)
{
    for (size_t i = 0; i < n; i++)
        if (inFramer.feed(buf[i]))
        {
            lastInboundFrameMs = millis();
            if (accept && rtcmToRx.write(inFramer.frame(), inFramer.frameLength()))
                inboundFrames++;
        }
}

bool outputsInboundRtcmActive() { return lastInboundFrameMs && millis() - lastInboundFrameMs < 10000; }

void outputsLoop(bool ntripStreaming, bool rtkAllowed)
{
    // New TCP clients
    if (tcpServer)
    {
        WiFiClient nc = tcpServer->accept();
        if (nc)
        {
            bool placed = false;
            for (auto &c : tcpClients)
                if (!c.connected())
                {
                    c.stop();
                    c = nc;
                    c.setNoDelay(true);
                    placed = true;
                    break;
                }
            if (!placed)
                nc.stop();
        }
    }

    // NMEA fan-out
    uint8_t buf[512];
    size_t n;
    while ((n = nmeaToNet.read(buf, sizeof(buf))) > 0)
    {
        if (useSpp && spp.hasClient())
            spp.write(buf, n);
        if (useBle && ble.connected())
            ble.write(buf, n);
        for (auto &c : tcpClients)
            if (c.connected())
                c.write(buf, n);
        sentBytes += n;
        monitorAdd(buf, n);
    }

    // Inbound RTCM from apps, used when the Postcard is not streaming NTRIP itself and the ladder allows RTK
    bool accept = cfg.inboundRtcm && !ntripStreaming && rtkAllowed && cfg.mode != "has";
    if (useSpp)
    {
        int a = spp.available();
        if (a > 0)
        {
            a = spp.readBytes(buf, a > (int)sizeof(buf) ? sizeof(buf) : a);
            inbound(buf, a, accept);
        }
    }
    if (useBle)
    {
        int a = ble.available();
        if (a > 0)
        {
            a = ble.readBytes(buf, a > (int)sizeof(buf) ? sizeof(buf) : a);
            inbound(buf, a, accept);
        }
    }
    for (auto &c : tcpClients)
    {
        int a = c.connected() ? c.available() : 0;
        if (a > 0)
        {
            a = c.read(buf, a > (int)sizeof(buf) ? sizeof(buf) : a);
            if (a > 0)
                inbound(buf, a, accept);
        }
    }
}

void outputsStatus(OutputStatus &s)
{
    s.sppConnected = useSpp && spp.hasClient();
    s.bleConnected = useBle && ble.connected();
    s.tcpClients = 0;
    for (auto &c : tcpClients)
        s.tcpClients += c.connected() ? 1 : 0;
    s.sentBytes = sentBytes;
    s.inboundFrames = inboundFrames;
}
