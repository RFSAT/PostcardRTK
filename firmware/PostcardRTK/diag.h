// diag.h - restart recorder. The main loop and the receiver task write a checkpoint number as they
// work; the numbers are kept across a restart in RTC memory that start-up does not clear, so after a
// watchdog restart or a crash the next run can say where the previous one stopped.
// Costs one RAM write per step; nothing is printed while the device runs normally.
#pragma once
#include <Arduino.h>
#include <stdint.h>

// Checkpoints. diagC0 (main loop): 1xx = start-up, 1-9 = main loop, 8x = history.
// diagC1 (receiver task): 2x = UART loop, 3x-4x = NMEA.
extern volatile uint32_t diagC0, diagC1;
static const uint32_t DIAG_CLEAN = 200; // set by Restart / Shut down from the page

void diagBoot(int resetReason);    // at start-up (esp_reset_reason()), before anything else writes a checkpoint
String diagText();                 // the previous run, one line (System tab, /api/status, USB at start-up)
bool diagProblem();                // the previous run ended in a fault or a watchdog restart
const char *diagStepName(int core, uint32_t c);
