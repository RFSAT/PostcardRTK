// eventlog.h - core 0: correction and system events kept in flash (/events.log), grouped by session
// (one session = one run from start-up to restart or power-off). Every event of the current session is
// kept; when the file reaches EV_MAX_BYTES, earlier sessions are trimmed first, oldest first.
#pragma once
#include <Arduino.h>

static const size_t EV_MAX_BYTES = 64u * 1024u;

void eventsSetup();             // after LittleFS.begin(): starts a new session
void eventAdd(const char *msg); // an event of this session (time-stamped here)
void eventsLoop();              // writes pending events to flash every 30 s
void eventsFlush();             // write now (restart, shut down)
uint32_t eventsSeq();           // increases with every event, so the page reloads the log only on change
uint32_t eventsSession();
// The whole log for the page: first line "C\t<current session>\t<seq>", then the file, then pending
void eventsText(String &out);
