// indicators.cpp - see indicators.h
#include "indicators.h"
#include "board.h"
#include "eventlog.h"
#include "methods.h"
#include "net.h"
#include "receiver.h"
#include "util.h"
#include <Arduino.h>

static uint32_t pressedAt = 0;
static bool apRequested = false;
static uint32_t lastRgb = 0xFFFFFFFF;

static void led(bool on)
{
    if (PIN_LED >= 0)
        digitalWrite(PIN_LED, on == LED_ACTIVE_HIGH ? HIGH : LOW);
}

static void errLed(bool on)
{
    if (PIN_ERR_LED >= 0)
        digitalWrite(PIN_ERR_LED, on == LED_ACTIVE_HIGH ? HIGH : LOW);
}

static void rgb(uint8_t r, uint8_t g, uint8_t b)
{
    if (PIN_RGB < 0)
        return;
    uint32_t c = ((uint32_t)r << 16) | ((uint32_t)g << 8) | b;
    if (c == lastRgb)
        return; // the WS2812 is written only on change
    lastRgb = c;
    rgbLedWrite(PIN_RGB, r, g, b);
}

void indicatorsSetup()
{
    if (PIN_LED >= 0)
        pinMode(PIN_LED, OUTPUT);
    led(false);
    if (PIN_ERR_LED >= 0)
        pinMode(PIN_ERR_LED, OUTPUT);
    errLed(false);
    if (PIN_BUTTON >= 0)
        pinMode(PIN_BUTTON, INPUT_PULLUP);
    rgb(0, 0, 0);
}

void indicatorsOff()
{
    led(false);
    errLed(false);
    rgb(0, 0, 0);
}

// Low brightness: the LEDs are seen through the case, and the WS2812 at full power is blinding
static const uint8_t B = 24;

static void methodColour()
{
    const uint32_t now = millis();
    const uint32_t at = currentMethodMs.load();
    Method m = (at && now - at < 5000) ? (Method)currentMethod.load() : M_NONE;
    switch (m)
    {
    case M_RTK_FIXED: rgb(0, B, 0); break;
    case M_RTK_FLOAT: rgb(B, B, 0); break;
    case M_HAS_DONE: rgb(0, B, B); break;
    case M_HAS_CONV: rgb(0, 0, B); break;
    case M_DGNSS:
    case M_SBAS: rgb(B, 0, B); break;
    case M_GNSS:
    case M_DR: rgb(B / 2, B / 2, B / 2); break;
    default:
        // red, blinking once a second: no position, or nothing from the receiver
        if ((now / 500) % 2)
            rgb(B, 0, 0);
        else
            rgb(0, 0, 0);
        break;
    }
}

static void wifiLed()
{
    const uint32_t now = millis();
    if (netUp())
        led(true);
    else if (netApMode())
        led((now / 125) % 2); // fast: setup network open
    else
        led((now / 1000) % 2); // slow: looking for a configured network
}

static void button()
{
    if (PIN_BUTTON < 0)
        return;
    const bool down = digitalRead(PIN_BUTTON) == LOW;
    if (!down)
    {
        pressedAt = 0;
        apRequested = false;
        return;
    }
    if (!pressedAt)
        pressedAt = millis() ? millis() : 1;
    if (!apRequested && millis() - pressedAt >= 3000)
    {
        apRequested = true; // once per press
        dbg("[button] held 3 s: opening the setup network\r\n");
        eventAdd("button held: setup access point opened");
        netStartAp();
    }
}

void indicatorsLoop()
{
    if (PIN_LED < 0 && PIN_RGB < 0 && PIN_BUTTON < 0 && PIN_ERR_LED < 0)
        return;
    button();
    wifiLed();
    methodColour();
    if (PIN_ERR_LED >= 0)
    {
        // error LED (F9R): on while there is no position or no data from the receiver
        const uint32_t at = currentMethodMs.load();
        const bool ok = at && millis() - at < 5000 && (Method)currentMethod.load() != M_NONE;
        errLed(!ok);
    }
}
