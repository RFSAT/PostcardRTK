// board.h - what differs between the boards PostcardRTK runs on. One source, one version for:
//  - SparkFun RTK Postcard (ESP32-PICO-MINI-02, LG290P on the board): variant V3 of the RFSAT RTK documentation
//  - Waveshare LG290P module + ESP32-C6 SuperMini joined by a UART: variant V2 (and V6 with the C6)
// The target is chosen by the build (tools/build.sh, BOARD=postcard | c6), not by a setting.
#pragma once
#include <sdkconfig.h>

#if CONFIG_IDF_TARGET_ESP32C6
#define BOARD_C6 1
#define BOARD_NAME "LG290P + ESP32-C6"
// C6 UART1 on the SuperMini's RX / TX pins <-> the LG290P module's UART3 socket (TXD3 / RXD3)
static const int PIN_GNSS_RX = 17;     // C6 RX <- LG290P TXD3
static const int PIN_GNSS_TX = 16;     // C6 TX -> LG290P RXD3
static const int PIN_GNSS_RESET = -1;  // not wired: the module resets itself at power-up
static const int PIN_LED = 15;         // on-board LED: Wi-Fi / setup network
static const int PIN_RGB = 8;          // on-board WS2812 RGB LED: positioning method
static const int PIN_BUTTON = 19;      // push-button to GND: hold 3 s for the setup network
static const bool LED_ACTIVE_HIGH = true;
// The C6 has Bluetooth LE only (no classic SPP)
#define BOARD_BT_CLASSIC 0
// No PSRAM, 512 KB RAM: smaller event log
#define BOARD_EVENT_LOG_BYTES (32u * 1024u)
#define BOARD_HIST_FILE_BYTES (256u * 1024u)
#else
#define BOARD_C6 0
#define BOARD_NAME "SparkFun RTK Postcard"
// SparkFun RTK Postcard: ESP32 <-> LG290P (as in SparkFun RTK Everywhere, Begin.ino)
static const int PIN_GNSS_RX = 21;    // ESP32 RX <- LG290P TX
static const int PIN_GNSS_TX = 22;    // ESP32 TX -> LG290P RX
static const int PIN_GNSS_RESET = 33; // high = run, low = reset
static const int PIN_LED = -1;        // no free LED or button on the Postcard on its own
static const int PIN_RGB = -1;
static const int PIN_BUTTON = -1;
static const bool LED_ACTIVE_HIGH = true;
#define BOARD_BT_CLASSIC 1
#define BOARD_EVENT_LOG_BYTES (64u * 1024u)
#define BOARD_HIST_FILE_BYTES (384u * 1024u)
#endif
