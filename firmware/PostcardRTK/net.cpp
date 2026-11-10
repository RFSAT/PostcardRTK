// net.cpp - main loop: Wi-Fi without blocking. Scans in the background, joins the first configured network
// it finds (in the order of the settings), rescans regularly while not connected (this is how the method
// ladder notices that a Wi-Fi network, and so RTK, has become available). A setup access point opens when
// no network is configured, or after 2 minutes without one, and closes again once the station is connected.
// Bluetooth and the receiver keep running meanwhile: nothing here waits for the radio.
#include "net.h"
#include "config.h"
#include "eventlog.h"
#include "util.h"
#include <ESPmDNS.h>
#include <WiFi.h>
#include <time.h>

enum class NetState : uint8_t
{
    Idle,     // not connected, waiting for the next scan
    Scanning, // background scan running
    Joining,  // WiFi.begin() issued
    Up,       // connected with an IP address
};

static NetState st = NetState::Idle;
static bool haveNetworks = false, apOn = false;
static uint32_t stateAt = 0, lastScanAt = 0, apForcedUntil = 0, bootAt = 0;
static bool firstScan = true;
static String joiningSsid;
static const uint32_t JOIN_TIMEOUT_MS = 20000;
static const uint32_t AP_AFTER_MS = 120000; // open the setup access point after this long without Wi-Fi

static String apName()
{
    uint8_t mac[6] = {0};
    WiFi.macAddress(mac);
    char s[40];
    snprintf(s, sizeof(s), "%s-Setup-%02X%02X", cfg.deviceName.c_str(), mac[4], mac[5]);
    return s;
}

static void apStart()
{
    if (apOn)
        return;
    WiFi.mode(WIFI_AP_STA);
    WiFi.softAP(apName().c_str(), cfg.apPass.c_str());
    apOn = true;
    dbg("[net] setup access point %s, http://%s\r\n", apName().c_str(), WiFi.softAPIP().toString().c_str());
    eventAdd(("setup access point " + apName() + " open").c_str());
}

static void apStop()
{
    if (!apOn)
        return;
    WiFi.softAPdisconnect(true);
    WiFi.mode(WIFI_STA);
    apOn = false;
    dbg("[net] setup access point closed\r\n");
}

void netStartAp()
{
    apForcedUntil = millis() + 600000; // 10 min, or as long as someone is connected to it
    if (!apForcedUntil)
        apForcedUntil = 1;
    apStart();
}

// http://<devicename>.local on the office network (Windows 10+, Android 12+, iOS, macOS)
static void startMdns()
{
    static bool started = false;
    if (started)
        return;
    String host = cfg.deviceName;
    host.toLowerCase();
    host.replace(" ", "-");
    if (MDNS.begin(host.c_str()))
    {
        MDNS.addService("http", "tcp", 80);
        started = true;
        dbg("[net] http://%s.local\r\n", host.c_str());
    }
}

static void setState(NetState s)
{
    st = s;
    stateAt = millis();
}

void netSetup()
{
    bootAt = millis();
    WiFi.persistent(false); // the settings live in config.json, not in the ESP32's own Wi-Fi store
    WiFi.setHostname(cfg.deviceName.c_str());
    WiFi.mode(WIFI_STA);
    WiFi.setAutoReconnect(false); // reconnection is done here, after a scan
    for (int i = 0; i < CFG_MAX_WIFI; i++)
        if (cfg.wifi[i].ssid.length())
            haveNetworks = true;
    dbg("[net] %s\r\n", haveNetworks ? "looking for the configured Wi-Fi" : "no Wi-Fi configured");
    if (!haveNetworks)
        apStart();
    setState(NetState::Idle);
}

static void startScan()
{
    WiFi.scanDelete();
    if (WiFi.scanNetworks(true /* async */, false) == WIFI_SCAN_FAILED)
    {
        lastScanAt = millis();
        return;
    }
    lastScanAt = millis();
    setState(NetState::Scanning);
}

// The first configured network (in the settings' order) that the scan found; -1 if none
static int pickNetwork(int found)
{
    for (int i = 0; i < CFG_MAX_WIFI; i++)
    {
        if (!cfg.wifi[i].ssid.length())
            continue;
        for (int k = 0; k < found; k++)
            if (WiFi.SSID(k) == cfg.wifi[i].ssid)
                return i;
    }
    return -1;
}

void netLoop()
{
    const uint32_t now = millis();

    // Setup access point: closes when the station is up and nobody uses it, unless opened on request
    bool forced = apForcedUntil && (int32_t)(apForcedUntil - now) > 0;
    if (apOn && st == NetState::Up && !forced && WiFi.softAPgetStationNum() == 0)
        apStop();
    if (!apOn && haveNetworks && st != NetState::Up && now - bootAt > AP_AFTER_MS)
        apStart();

    if (!haveNetworks)
        return;

    switch (st)
    {
    case NetState::Idle:
        if (firstScan || now - lastScanAt >= cfg.wifiScanSec * 1000)
        {
            firstScan = false;
            startScan();
        }
        break;

    case NetState::Scanning: {
        int16_t r = WiFi.scanComplete();
        if (r == WIFI_SCAN_RUNNING)
        {
            if (now - stateAt > 15000) // stuck scan
            {
                WiFi.scanDelete();
                setState(NetState::Idle);
            }
            break;
        }
        int pick = r > 0 ? pickNetwork(r) : -1;
        WiFi.scanDelete();
        if (pick < 0)
        {
            setState(NetState::Idle);
            break;
        }
        joiningSsid = cfg.wifi[pick].ssid;
        dbg("[net] joining %s\r\n", joiningSsid.c_str());
        WiFi.begin(cfg.wifi[pick].ssid.c_str(), cfg.wifi[pick].pass.length() ? cfg.wifi[pick].pass.c_str() : nullptr);
        setState(NetState::Joining);
        break;
    }

    case NetState::Joining:
        if (WiFi.status() == WL_CONNECTED && WiFi.localIP() != IPAddress(0, 0, 0, 0))
        {
            setState(NetState::Up);
            dbg("[net] connected to %s, %s\r\n", WiFi.SSID().c_str(), WiFi.localIP().toString().c_str());
            eventAdd(("Wi-Fi connected to " + WiFi.SSID() + ", " + WiFi.localIP().toString()).c_str());
            startMdns();
            static bool ntp = false;
            if (!ntp)
            {
                configTime(0, 0, "pool.ntp.org", "time.google.com"); // UTC, for the bench GGA
                ntp = true;
            }
        }
        else if (now - stateAt > JOIN_TIMEOUT_MS)
        {
            dbg("[net] could not join %s\r\n", joiningSsid.c_str());
            WiFi.disconnect(false);
            setState(NetState::Idle);
        }
        break;

    case NetState::Up:
        if (WiFi.status() != WL_CONNECTED)
        {
            dbg("[net] Wi-Fi lost\r\n");
            eventAdd("Wi-Fi lost, scanning for the configured networks");
            WiFi.disconnect(false);
            firstScan = true; // scan again straight away
            setState(NetState::Idle);
        }
        break;
    }
}

bool netUp() { return st == NetState::Up && WiFi.status() == WL_CONNECTED; }
bool netApMode() { return apOn && !netUp(); }
String netSsid() { return netUp() ? WiFi.SSID() : (apOn ? apName() : String("")); }
String netIp() { return netUp() ? WiFi.localIP().toString() : (apOn ? WiFi.softAPIP().toString() : String("")); }
int netRssi() { return netUp() ? WiFi.RSSI() : 0; }
const char *netStateName()
{
    switch (st)
    {
    case NetState::Scanning: return "scanning";
    case NetState::Joining: return "joining";
    case NetState::Up: return "connected";
    default: return haveNetworks ? "no configured network in range" : "no network configured";
    }
}
