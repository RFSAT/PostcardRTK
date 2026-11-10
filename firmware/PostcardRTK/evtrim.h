// evtrim.h - trimming rule for the event log (portable, unit-tested on the PC).
// Lines are "S\t<session>\t..." (session start) or "E\t<session>\t..." (event), each ending in '\n'.
// To fit maxBytes: drop whole earlier sessions' lines first, oldest first; only when the current
// session alone is too big, drop its oldest events and put one "D\t<session>\t<count>" line in
// their place, so the page can say how many were dropped.
#pragma once
#include <cstddef>
#include <cstdint>
#include <string>

// Returns the trimmed text (unchanged when it already fits).
std::string evTrim(const std::string &log, uint32_t currentSession, size_t maxBytes);
// Session number of a line (0 when it has none)
uint32_t evSession(const char *line, size_t len);
