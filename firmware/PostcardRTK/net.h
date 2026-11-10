// net.h - main loop: Wi-Fi station / setup access point (never blocks)
#pragma once
#include <Arduino.h>

void netSetup();
void netLoop();
void netStartAp(); // button: open the setup access point now
bool netUp();      // station connected (internet assumed)
bool netApMode();
String netSsid();
String netIp();
int netRssi();
const char *netStateName(); // scanning, joining, connected, no configured network in range
