// ladder.h - decides whether the rover should be using RTK corrections, in the order of preference
// RTK (NTRIP over Wi-Fi, or RTCM from a phone app) > Galileo HAS > SBAS (EGNOS) > GNSS only.
//
// The LG290P computes one solution at a time and falls back on its own: when RTCM stops it keeps RTK for
// up to its differential timeout, then uses Galileo HAS (if E6 is received) or SBAS, then GNSS only.
// So the only lever the firmware has is whether corrections are fed to it. The ladder:
//   - feeds RTK corrections whenever a source is available (configured Wi-Fi in range, or an app sending RTCM);
//   - gives RTK a trial period; if in that time RTK never gets better than what HAS/SBAS achieved here
//     recently (or than a typical SBAS result when neither has been seen), it pauses RTK so the receiver
//     falls back to HAS/SBAS, and tries RTK again later (15 min, doubling up to 1 h on repeated failures);
//   - reports what it decided and why, in plain words, for the web page, the event log and the
//     $PRFSM status sentence sent to Bluetooth clients.
// Positions are never held or smoothed: they go out exactly as the receiver computes them.
// Portable (no Arduino calls), unit-tested on the PC.
#pragma once
#include "methods.h"
#include <cstddef>
#include <cstdint>

enum LadderState : uint8_t
{
    LS_NO_SOURCE = 0, // no correction source at the moment: the receiver uses HAS, SBAS or GNSS on its own
    LS_RTK_TRYING,    // corrections are arriving, RTK not (yet) in use
    LS_RTK,           // RTK float or fixed with live corrections
    LS_RTK_PAUSED,    // RTK did not perform: corrections held back so HAS/SBAS can work, retried later
    LS_HAS_ONLY,      // settings: never use RTK corrections
};
const char *ladderStateName(LadderState s); // "no source", "trying RTK", "RTK", "RTK paused", "HAS only"
const char *ladderStateCode(LadderState s); // NOSRC, TRY, RTK, PAUSE, HASONLY (for $PRFSM)

struct LadderConfig
{
    bool rtkAllowed = true;       // false = "HAS only" mode
    uint32_t trialSec = 300;      // RTK has this long to perform before it may be paused
    float rtkGoodM = 0.5f;        // RTK float with the receiver's estimate at or below this counts as performing
    float altAssumeM = 1.5f;      // what SBAS typically gives, used when HAS/SBAS has not been seen here yet
    uint32_t pauseSec = 900;      // first pause; doubles after each failed retry
    uint32_t maxPauseSec = 3600;
    uint32_t altMemorySec = 3600; // a HAS/SBAS result counts as "what the alternative achieves" this long
    uint32_t fixedResetSec = 60;  // RTK fixed this long clears the pause doubling
};

struct LadderInput
{
    uint32_t nowMs = 0;
    bool sourceAvailable = false; // Wi-Fi joined and a caster configured, or an app sending RTCM
    bool correctionsLive = false; // RTCM reached the receiver within the last 5 s
    Method method = M_NONE;       // what the receiver is doing now (classifyMethod)
    float estH = -1;              // the receiver's horizontal estimate for it, m (-1 unknown)
};

class Ladder
{
  public:
    void configure(const LadderConfig &c) { _c = c; }
    // Call once per receiver epoch (every GGA). Returns true when RTK corrections may be used now.
    bool update(const LadderInput &in);
    LadderState state() const { return _state; }
    bool allowRtk() const { return _allow; }
    const char *reason() const { return _reason; }
    // The reason changed since the last call (for the event log)
    bool takeChange();
    uint32_t pausedUntilMs() const { return _pausedUntil; }
    float altBest() const { return _altBest; }       // best HAS converged / SBAS estimate remembered, -1 none
    int pauseCount() const { return _pauses; }

  private:
    void setReason(const char *r);
    LadderConfig _c;
    LadderState _state = LS_NO_SOURCE;
    bool _allow = true;
    char _reason[160] = "starting";
    bool _changed = true;
    uint32_t _trialStart = 0, _lastGood = 0, _fixedSince = 0, _pausedUntil = 0;
    bool _paused = false, _inTrial = false;
    int _level = 0, _pauses = 0;
    float _altBest = -1;
    uint32_t _altAt = 0;
    float _lastRtkEst = -1;
};

// Short method code for $PRFSM: FIX, FLOAT, DGNSS, HASCONV, HAS, SBAS, GNSS, DR, NONE
const char *methodCode(Method m);

// $PRFSM status sentence, sent to Bluetooth/TCP clients once per epoch after the GGA:
//   $PRFSM,<ladder state>,<method>,<corrections live|stale|none>,<age s>,<station>,<estH m>,<estV m>*hh
// Empty fields where unknown. Returns the length (0 if it does not fit).
size_t prfsmSentence(char *out, size_t outSize, LadderState s, Method m, const CorrInfo &c, float estH, float estV);
