// indicators.h - main loop: status LEDs and the setup button, on boards that have them (board.h).
// ESP32-C6 SuperMini (V2):
//  - on-board LED (GPIO15): fast blink = setup network open, slow blink = searching for Wi-Fi, on = Wi-Fi connected
//  - on-board RGB LED (GPIO8), positioning method: green RTK fixed, yellow RTK float, cyan Galileo HAS converged,
//    blue HAS converging, magenta DGNSS / SBAS, white GNSS only, red no position or no data from the receiver
//  - button GPIO19 to GND: hold 3 s to open the setup network (as GP16 on PicoRTK)
// AgriBIT receiver (F9R): "ESP_RTK OK" LED (GPIO15) as the Wi-Fi LED above, "ESP_RTK ERROR" LED (GPIO12) on while there
// is no position or no data from the receiver, top button (GPIO13) hold 3 s for the setup network.
// The Postcard on its own has none of these: the functions do nothing there.
#pragma once

void indicatorsSetup();
void indicatorsLoop();
void indicatorsOff(); // before shutting down
