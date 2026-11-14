// display.h - optional OLED status screen (V7: SSD1306 0.96" 128x64 on I2C 26/27). Does nothing on boards without one,
// or when no display answers at start-up.
#pragma once

void displaySetup(const char *version); // boot screen: RFSAT Limited, board, firmware version
void displayLoop();                     // status screen, redrawn twice a second
void displayOff();
