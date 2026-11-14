// config.h - settings stored as /config.json in LittleFS
#pragma once
#include "board.h"
#include <Arduino.h>

static const int CFG_MAX_WIFI = 4;
static const int CFG_MAX_SOURCES = 4;
static const int CFG_MAX_BOOTCMDS = 8;

struct WifiNet
{
    String ssid, pass;
};

struct NtripSource
{
    String name;  // "Own base EOS"
    String type;  // base | vrs | near | other
    String host;
    uint16_t port = 2101;
    String mount, user, pass;
    bool gga = false; // send our position (VRS / nearest station)
    bool v2 = false;  // NTRIP 2.0 request
    bool tls = false; // TLS (certificate not verified)
    bool valid() const { return host.length() && port && mount.length(); }
};

struct Config
{
#if defined(BOARD_V7)
    String deviceName = "RFSAT-V7";
#else
    String deviceName = "PostcardRTK";
#endif
    String adminPass = "postcardrtk"; // web page login (user "admin")
    String apPass = "postcardrtk-setup"; // setup access point, 8+ characters

    WifiNet wifi[CFG_MAX_WIFI];
    NtripSource src[CFG_MAX_SOURCES];

    // Correction mode: "rtk" = automatic: RTK corrections whenever a source is available (NTRIP over Wi-Fi, or
    //                         RTCM from a Bluetooth/TCP app), judged by the method ladder; otherwise the receiver
    //                         uses Galileo HAS, SBAS, GNSS only (ladder.h)
    //                  "has" = never RTK: Galileo HAS from the satellites, SBAS, GNSS only (no internet needed)
    String mode = "rtk";
    // Method ladder: RTK gets trialSec to perform (fixed, or float within rtkGoodM) before it may be paused
    // for pauseSec (doubling after each failed retry, up to 1 h) when the fallback does better
    uint32_t ladderTrialSec = 300;
    float ladderGoodM = 0.5f;
    uint32_t ladderPauseSec = 900;
    bool prfsm = true; // $PRFSM status sentence to Bluetooth/TCP clients after every GGA
    bool galileoHas = true; // Galileo HAS (PPP) enabled in the receiver; always on in "has" mode

    // Reference point (surveyed antenna position, e.g. recorded from an RTK fixed solution):
    // the Home page shows each method's actual error against it
    bool refSet = false;
    double refLat = 0, refLon = 0;
    float refAlt = 0; // m above mean sea level, as in GGA

    // Failover
    bool failover = true;
    uint32_t failbackSec = 600;  // return to source 1 (0 = never)
    uint32_t retryMs = 2000;     // delay before the next source
    uint32_t backoffSec[5] = {15, 30, 60, 120, 300};
    uint32_t dataTimeoutSec = 30; // no valid RTCM frame for this long = source failure
    uint32_t maxAgeSec = 10;      // GGA correction age above this while streaming = failure
    uint32_t ggaSec = 10;         // GGA upload interval
    uint32_t wifiScanSec = 60;    // while not on Wi-Fi: look for the configured networks this often (RTK check)
    // Bench position: sent as GGA to VRS/nearest casters while the receiver has no fix (0,0 = off)
    double benchLat = 0, benchLon = 0;
    float benchAlt = 0;

    // Outputs
#if defined(BOARD_V7)
    String bt = "spp";   // V7 (no PSRAM): Bluetooth Classic only by default, as the original AgriBIT receiver; both possible
#else
    String bt = "both";  // spp | ble | both | off (Android apps use SPP, iPhone apps BLE)
#endif
    bool usb = false;     // receiver stream on USB (also bridges USB input to the receiver)
    uint16_t tcpPort = 2947; // NMEA TCP server, 0 = off
    bool inboundRtcm = true; // accept RTCM from Bluetooth/TCP clients when NTRIP is not streaming
    String nmeaTypes;        // sentence types sent to Bluetooth/TCP, comma-separated ("GGA,RMC,GST"); empty = all

    // Receiver
    uint32_t uartBaud = 460800;
    String bootCmd[CFG_MAX_BOOTCMDS]; // sent to the LG290P after start-up ($...; checksum added if missing)
};

extern Config cfg;

bool configLoad();                       // from flash, defaults when missing
bool configSave();                       // to flash
String configToJson(bool withSecrets);   // secrets replaced by "********" unless requested
bool configFromJson(const String &json, String &err); // merges; "********" keeps the stored secret
bool configReset();                // delete the saved settings: the next start uses the built-in defaults
bool configHasPrivateDefaults();   // built with tools/make_defaults.py (Wi-Fi and logins compiled in)
