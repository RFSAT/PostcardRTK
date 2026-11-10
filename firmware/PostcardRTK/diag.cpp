// diag.cpp - see diag.h. ESP32: the checkpoints live in RTC memory that a software, watchdog or crash
// restart does not clear; the reset reason says which of those it was.
#include "diag.h"
#include <esp_attr.h>
#include <esp_system.h>

static RTC_NOINIT_ATTR uint32_t dMagic;
RTC_NOINIT_ATTR volatile uint32_t diagC0;
RTC_NOINIT_ATTR volatile uint32_t diagC1;
static const uint32_t MAGIC = 0xD1A6D1A6;
static String prev;
static bool problem = false;

const char *diagStepName(int core, uint32_t c)
{
    if (core == 0)
        switch (c)
        {
        case 0: return "idle";
        case 1: return "Wi-Fi";
        case 2: return "network state";
        case 3: return "web server";
        case 4: return "NTRIP client";
        case 5: return "Bluetooth/TCP outputs";
        case 6: return "status";
        case 8: return "history";
        case 9: return "end of main loop";
        case 81: return "history: taking records from the receiver task";
        case 83: return "history: records taken";
        case 84: return "history: writing flash";
        case 101: return "start-up: shared data";
        case 102: return "start-up: flash file system";
        case 103: return "start-up: loading settings";
        case 104: return "start-up: reference point";
        case 105: return "start-up: history";
        case 106: return "start-up: settings loaded";
        case 107: return "start-up: status";
        case 108: return "start-up: Bluetooth";
        case 109: return "start-up: Wi-Fi";
        case 110: return "start-up: web server";
        case 111: return "start-up: NTRIP";
        case 112: return "start-up done";
        case DIAG_CLEAN: return "restart requested";
        default: return "?";
        }
    switch (c)
    {
    case 0: return "idle";
    case 1: return "UART loop";
    case 20: return "NMEA framing";
    case 21: return "UART loop";
    case 30: return "NMEA sentence";
    case 31: return "NMEA filter";
    case 32: return "NMEA counters";
    case 33: return "accuracy messages";
    case 34: return "method";
    case 35: return "waiting for shared data";
    case 36: return "method tracker";
    case 37: return "after method tracker";
    case 38: return "history record";
    case 39: return "history record queued";
    case 40: return "NMEA output";
    default: return "?";
    }
}

void diagBoot(int resetReason)
{
    char b[220];
    const uint32_t c0 = diagC0, c1 = diagC1;
    const bool known = dMagic == MAGIC;
    const esp_reset_reason_t r = (esp_reset_reason_t)resetReason;
    if (known && (r == ESP_RST_PANIC))
    {
        snprintf(b, sizeof(b), "crashed; main loop at %lu %s, receiver task at %lu %s", (unsigned long)c0,
                 diagStepName(0, c0), (unsigned long)c1, diagStepName(1, c1));
        problem = true;
    }
    else if (known && c0 == DIAG_CLEAN)
        snprintf(b, sizeof(b), "ended normally (restart or shut down from the page)");
    else if (known && (r == ESP_RST_TASK_WDT || r == ESP_RST_INT_WDT || r == ESP_RST_WDT))
    {
        snprintf(b, sizeof(b), "hung, restarted by the watchdog; main loop at %lu %s, receiver task at %lu %s",
                 (unsigned long)c0, diagStepName(0, c0), (unsigned long)c1, diagStepName(1, c1));
        problem = true;
    }
    else if (known && r == ESP_RST_BROWNOUT)
    {
        snprintf(b, sizeof(b), "power dipped (brown-out): check the USB supply and cable");
        problem = true;
    }
    else if (known)
        snprintf(b, sizeof(b), "ended without a fault (power removed, reset button or new firmware)");
    else
        snprintf(b, sizeof(b), "nothing recorded (power-up)");
    prev = b;
    diagC0 = diagC1 = 0;
    dMagic = MAGIC;
}

String diagText() { return prev; }
bool diagProblem() { return problem; }
