// selector.h - correction-source selection and failover (portable, unit-tested on the PC)
//
// Same policy as the RFSAT ESP32 firmware:
//   soft failure  -> next usable source after retryMs
//   hard failure  -> the source is disabled until settings change, then next source
//   every usable source failed in a row -> back-off (15 s, 30 s, 1 min, 2 min, 5 min...)
//                                         then start again from the highest priority source
//   all sources disabled -> stopped
//   running on a lower-priority source for failbackSec -> return to the best source
#pragma once
#include <cstdint>

static const int SEL_MAX_SOURCES = 8;
static const int SEL_MAX_BACKOFF = 8;

struct SelectorDecision
{
    int source = 0;       // source to use next
    uint32_t delayMs = 0; // wait this long before connecting
    bool switched = false;
    bool backoff = false; // a full cycle failed
    bool stopped = false; // nothing usable is left
};

class SourceSelector
{
  public:
    SourceSelector();

    // Configuration
    void setSources(int count, const bool *valid); // valid[i]: host, port and mount set
    void setFailover(bool on) { _failover = on; }
    void setFailbackSec(uint32_t s) { _failbackSec = s; }
    void setRetryMs(uint32_t ms) { _retryMs = ms; }
    void setBackoff(const uint32_t *sec, int n);

    // Events
    SelectorDecision fail(bool hard, uint32_t nowMs);
    void connected(uint32_t nowMs);
    bool failbackDue(uint32_t nowMs) const;
    void reset(bool clearDisabled); // back to the highest priority source

    // State
    int active() const { return _active; }
    bool usable(int i) const;
    int usableCount() const;
    int validCount() const;
    int firstUsable() const;
    bool disabled(int i) const { return (_disabledMask >> i) & 1; }
    uint32_t switches() const { return _switches; }
    int backoffLevel() const { return _backoffLevel; }

  private:
    bool valid(int i) const;
    int _count = 0;
    bool _valid[SEL_MAX_SOURCES] = {};
    uint32_t _disabledMask = 0;
    int _active = 0;
    int _cycleFailures = 0;
    int _backoffLevel = 0;
    bool _failover = true;
    uint32_t _failbackSec = 600;
    uint32_t _retryMs = 2000;
    uint32_t _backoffSec[SEL_MAX_BACKOFF];
    int _backoffCount = 0;
    uint32_t _connectedAt = 0;
    bool _isConnected = false;
    uint32_t _switches = 0;
};
