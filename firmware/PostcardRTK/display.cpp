// display.cpp - see display.h
#include "display.h"
#include "board.h"

#if defined(BOARD_F9R)
#include "config.h"
#include "net.h"
#include "ntrip.h"
#include "outputs.h"
#include "receiver.h"
#include <Arduino.h>
#include <U8g2lib.h>
#include <WiFi.h>
#include <Wire.h>

static U8G2_SSD1306_128X64_NONAME_F_HW_I2C oled(U8G2_R0, U8X8_PIN_NONE);
static bool present = false;
static uint32_t lastDraw = 0, bootUntil = 0;

// Casters shown by name (the same list as the RFSAT RTK Everywhere firmware's C: row); any other by its host
static const char *casterName(const String &host)
{
    static const struct
    {
        const char *host, *name;
    } known[] = {{"gr.nrtk.eu", "Metrica SmartNet"}, {"euref-ip.net", "EUREF-IP"}, {"www.euref-ip.net", "EUREF-IP"},
                 {"rtk2go.com", "RTK2go"}, {"caster.centipede.fr", "Centipede"}};
    for (auto &k : known)
        if (host.equalsIgnoreCase(k.host))
            return k.name;
    return host.c_str();
}

static const char *shortMethod(Method m)
{
    switch (m)
    {
    case M_RTK_FIXED: return "RTK FIXED";
    case M_RTK_FLOAT: return "RTK FLOAT";
    case M_DGNSS: return "DGNSS";
    case M_SBAS: return "SBAS";
    case M_GNSS: return "GNSS";
    case M_DR: return "DEAD RECKON";
    default: return "NO FIX";
    }
}

void displaySetup(const char *version)
{
    Wire.begin(PIN_OLED_SDA, PIN_OLED_SCL, 400000);
    Wire.beginTransmission(0x3C);
    present = Wire.endTransmission() == 0; // the display is optional
    if (!present)
        return;
    oled.begin();
    oled.clearBuffer();
    oled.setFont(u8g2_font_helvB12_tr);
    const char *t1 = "RFSAT";
    oled.drawStr((128 - oled.getStrWidth(t1)) / 2, 18, t1);
    oled.setFont(u8g2_font_6x10_tr);
    const char *t2 = "L I M I T E D";
    oled.drawStr((128 - oled.getStrWidth(t2)) / 2, 31, t2);
    oled.drawHLine(14, 35, 100);
    oled.drawStr((128 - oled.getStrWidth("RTK F9R")) / 2, 48, "RTK F9R");
    char v[24];
    snprintf(v, sizeof(v), "v%s", version);
    oled.drawStr((128 - oled.getStrWidth(v)) / 2, 61, v);
    oled.sendBuffer();
    bootUntil = millis() + 3000;
}

void displayOff()
{
    if (!present)
        return;
    oled.clearBuffer();
    oled.sendBuffer();
    oled.setPowerSave(1);
}

// Rows (6x10 font, 21 characters): status, accuracy, C: caster, M: mount point, corrections, network
void displayLoop()
{
    if (!present)
        return;
    const uint32_t now = millis();
    if (now < bootUntil || now - lastDraw < 500)
        return;
    lastDraw = now;
    EpochNow e;
    getEpochNow(e);
    const bool fresh = e.atMs && now - e.atMs < 3000;
    NtripStatus ns;
    ntripStatus(ns);
    OutputStatus os;
    outputsStatus(os);
    char line[40];

    oled.clearBuffer();
    oled.setFont(u8g2_font_6x10_tr);
    // 1: method and satellites, Bluetooth client marker
    snprintf(line, sizeof(line), "%-11s %2dsv %s", fresh ? shortMethod(e.method) : "NO DATA", fresh ? e.sats : 0,
             (os.sppConnected || os.bleConnected) ? "BT" : "");
    oled.drawStr(0, 9, line);
    // 2: receiver's accuracy estimate
    if (fresh && e.estH >= 0)
    {
        if (e.estH < 1)
            snprintf(line, sizeof(line), "H %.3f m  V %.3f m", e.estH, e.estV >= 0 ? e.estV : 0);
        else
            snprintf(line, sizeof(line), "H %.2f m  V %.2f m", e.estH, e.estV >= 0 ? e.estV : 0);
    }
    else
        snprintf(line, sizeof(line), "accuracy -");
    oled.drawStr(0, 20, line);
    // 3-4: caster and mount point while the rover's own NTRIP client streams
    if (ns.state == NtripState::Streaming && ns.source >= 0 && ns.source < CFG_MAX_SOURCES)
    {
        const NtripSource &src = cfg.src[ns.source];
        snprintf(line, sizeof(line), "C:%s", casterName(src.host));
        line[21] = 0;
        oled.drawStr(0, 31, line);
        snprintf(line, sizeof(line), "M:%s", src.mount.c_str());
        line[21] = 0;
        oled.drawStr(0, 42, line);
    }
    else
    {
        snprintf(line, sizeof(line), "NTRIP %s", ntripStateName(ns.state));
        line[21] = 0;
        oled.drawStr(0, 31, line);
    }
    // 5: corrections in use
    if (fresh && e.corr.station >= 0)
        snprintf(line, sizeof(line), "%s st %d age %.0fs", e.corr.live ? "live" : "stale", e.corr.station, e.corr.age >= 0 ? e.corr.age : 0);
    else
        line[0] = 0;
    oled.drawStr(0, 53, line);
    // 6: network
    if (netApMode())
        snprintf(line, sizeof(line), "SETUP %s", WiFi.softAPSSID().c_str());
    else if (netUp())
        snprintf(line, sizeof(line), "%s", netIp().c_str());
    else
        snprintf(line, sizeof(line), "Wi-Fi %s", netStateName());
    line[21] = 0;
    oled.drawStr(0, 64, line);
    oled.sendBuffer();
}

#else
void displaySetup(const char *) {}
void displayLoop() {}
void displayOff() {}
#endif
