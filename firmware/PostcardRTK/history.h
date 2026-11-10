// history.h - core 0: positioning history in flash (/hist2a.bin, /hist2b.bin), kept across restarts
#pragma once
#include "methods.h"
#include "sky.h"
#include <Arduino.h>

void historySetup();              // after LittleFS.begin(); writes a restart marker
void historyLoop();               // drains core 1's records, appends to flash every 5 min
void historyFlush();              // write what is buffered now (shutdown, restart, download)
bool historyClear();
uint32_t historyRecords();        // in flash + buffered
uint32_t historyBytes();
void historyInfo(uint32_t &firstYmd, uint32_t &firstMs, uint32_t &lastYmd, uint32_t &lastMs);
// Stream records newest first (RAM buffer, then the files from their ends); return false to stop
void historyBackward(bool (*fn)(const HistRec &, void *), void *ctx);
// Stream every record, oldest first; return false from the callback to stop
void historyForEach(bool (*fn)(const HistRec &, void *), void *ctx);
// Last known sky from the previous run (loaded at start-up, saved with every history write)
const SkySnap &skyLastKnown();
static const uint32_t HIST_FILE_MAX = 384u * 1024u; // two files: about 30 h at one record every 10 s
