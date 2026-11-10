// ladder.cpp - see ladder.h
#include "ladder.h"
#include "nmea.h"
#include <cstdio>
#include <cstring>

const char *ladderStateName(LadderState s)
{
    switch (s)
    {
    case LS_RTK_TRYING: return "trying RTK";
    case LS_RTK: return "RTK";
    case LS_RTK_PAUSED: return "RTK paused";
    case LS_HAS_ONLY: return "HAS only";
    default: return "no correction source";
    }
}

const char *ladderStateCode(LadderState s)
{
    switch (s)
    {
    case LS_RTK_TRYING: return "TRY";
    case LS_RTK: return "RTK";
    case LS_RTK_PAUSED: return "PAUSE";
    case LS_HAS_ONLY: return "HASONLY";
    default: return "NOSRC";
    }
}

const char *methodCode(Method m)
{
    switch (m)
    {
    case M_RTK_FIXED: return "FIX";
    case M_RTK_FLOAT: return "FLOAT";
    case M_DGNSS: return "DGNSS";
    case M_HAS_CONV: return "HASCONV";
    case M_HAS_DONE: return "HAS";
    case M_SBAS: return "SBAS";
    case M_GNSS: return "GNSS";
    case M_DR: return "DR";
    default: return "NONE";
    }
}

void Ladder::setReason(const char *r)
{
    if (strncmp(r, _reason, sizeof(_reason)) == 0)
        return;
    strncpy(_reason, r, sizeof(_reason) - 1);
    _reason[sizeof(_reason) - 1] = 0;
    _changed = true;
}

bool Ladder::takeChange()
{
    bool c = _changed;
    _changed = false;
    return c;
}

bool Ladder::update(const LadderInput &in)
{
    const uint32_t now = in.nowMs;
    char r[160];

    // What the fallback achieves here: the best HAS-converged or SBAS estimate within the memory window
    if ((in.method == M_HAS_DONE || in.method == M_SBAS) && in.estH > 0)
    {
        if (_altBest < 0 || in.estH < _altBest || now - _altAt > _c.altMemorySec * 1000)
        {
            _altBest = in.estH;
            _altAt = now;
        }
    }
    else if (_altBest >= 0 && now - _altAt > _c.altMemorySec * 1000)
        _altBest = -1;

    if (!_c.rtkAllowed)
    {
        _state = LS_HAS_ONLY;
        _allow = false;
        _paused = _inTrial = false;
        setReason("RTK is switched off in the settings: Galileo HAS, then SBAS, then GNSS only");
        return _allow;
    }

    if (_paused)
    {
        if ((int32_t)(now - _pausedUntil) < 0)
        {
            _state = LS_RTK_PAUSED;
            _allow = false;
            return _allow; // the reason was set when pausing
        }
        _paused = false;
        _inTrial = false;
        setReason("trying RTK again after the pause");
    }

    if (!in.sourceAvailable)
    {
        _state = LS_NO_SOURCE;
        _allow = true; // use RTK as soon as a source appears
        _inTrial = false;
        _fixedSince = 0;
        setReason("no correction source (no configured Wi-Fi in range, no app sending corrections): "
                  "the receiver uses Galileo HAS, SBAS or GNSS on its own");
        return _allow;
    }

    _allow = true;
    if (!_inTrial)
    {
        _inTrial = true;
        _trialStart = now;
        _lastGood = 0;
        _fixedSince = 0;
    }

    if (!in.correctionsLive)
    {
        // Wi-Fi is up or an app is connected, but no corrections reach the receiver (caster down, wrong mount
        // point, app not sending): nothing to judge yet, so the trial only counts time with live corrections
        _state = LS_RTK_TRYING;
        _trialStart = now;
        _fixedSince = 0;
        setReason("correction source connected, waiting for corrections");
        return _allow;
    }
    const bool rtk = in.method == M_RTK_FIXED || in.method == M_RTK_FLOAT;
    if (rtk)
        _lastRtkEst = in.estH;
    if (in.method == M_RTK_FIXED)
    {
        if (!_fixedSince)
            _fixedSince = now ? now : 1;
        if (now - _fixedSince >= _c.fixedResetSec * 1000)
            _level = 0; // RTK works here: a later problem starts from the short pause again
    }
    else
        _fixedSince = 0;

    const bool performing = rtk && (in.method == M_RTK_FIXED || (in.estH >= 0 && in.estH <= _c.rtkGoodM));
    if (performing)
        _lastGood = now ? now : 1;

    _state = rtk ? LS_RTK : LS_RTK_TRYING;
    const uint32_t trialMs = _c.trialSec * 1000;
    const uint32_t inTrialMs = now - _trialStart;
    if (performing)
    {
        if (in.method == M_RTK_FIXED)
            setReason("RTK fixed");
        else
        {
            snprintf(r, sizeof(r), "RTK float, estimated within %.1f m", _c.rtkGoodM);
            setReason(r);
        }
        return _allow;
    }

    // Not performing. Within the trial, or within a trial period of the last good RTK: keep trying.
    const bool sinceGood = _lastGood && now - _lastGood < trialMs;
    if (inTrialMs < trialMs || sinceGood)
    {
        if (rtk)
            setReason("RTK float, converging");
        else
            setReason("corrections arriving, waiting for RTK");
        return _allow;
    }

    // Trial over, nothing good for a whole trial period. Pause only if the fallback is likely better.
    const float alt = _altBest >= 0 ? _altBest : _c.altAssumeM;
    const char *altWhat = _altBest >= 0 ? "HAS/SBAS reached" : "SBAS typically gives";
    bool pause = false;
    if (!rtk)
    {
        pause = true; // corrections for a whole trial and the receiver never got to RTK
        snprintf(r, sizeof(r), "RTK paused for %lu min: corrections for %lu min but no RTK solution",
                 0ul, (unsigned long)(inTrialMs / 60000));
    }
    else if (in.estH >= 0 && in.estH > alt)
    {
        pause = true;
        snprintf(r, sizeof(r), "RTK paused for %lu min: RTK float stayed at %.1f m for %lu min, %s %.1f m", 0ul,
                 in.estH, (unsigned long)(inTrialMs / 60000), altWhat, alt);
    }
    if (!pause)
    {
        setReason(in.estH >= 0 ? "RTK float, not converging, but still better than the fallback"
                               : "RTK float, the receiver gives no estimate");
        return _allow;
    }

    uint32_t sec = _c.pauseSec;
    for (int i = 0; i < _level && sec < _c.maxPauseSec; i++)
        sec *= 2;
    if (sec > _c.maxPauseSec)
        sec = _c.maxPauseSec;
    if (_level < 8)
        _level++;
    _pauses++;
    _paused = true;
    _pausedUntil = now + sec * 1000;
    _state = LS_RTK_PAUSED;
    _allow = false;
    _inTrial = false;
    // put the pause length into the text prepared above ("for 0 min")
    char final[160];
    const char *p = strstr(r, "for 0 min");
    if (p)
    {
        size_t head = (size_t)(p - r);
        snprintf(final, sizeof(final), "%.*sfor %lu min%s", (int)head, r, (unsigned long)(sec / 60), p + 9);
    }
    else
        snprintf(final, sizeof(final), "%s", r);
    setReason(final);
    return _allow;
}

static void fmtNum(char *o, size_t n, float v, int dec)
{
    if (v < 0)
        o[0] = 0;
    else
        snprintf(o, n, "%.*f", dec, v);
}

size_t prfsmSentence(char *out, size_t outSize, LadderState s, Method m, const CorrInfo &c, float estH, float estV)
{
    const int g = methodGroup(m);
    const char *corr = (g < 0 || g == G_GNSS) ? "none" : c.live ? "live" : "stale";
    char age[12], st[8] = "", eh[12], ev[12];
    fmtNum(age, sizeof(age), c.age, 1);
    if (c.station >= 0)
        snprintf(st, sizeof(st), "%d", c.station);
    fmtNum(eh, sizeof(eh), estH, 3);
    fmtNum(ev, sizeof(ev), estV, 3);
    char body[128];
    snprintf(body, sizeof(body), "$PRFSM,%s,%s,%s,%s,%s,%s,%s", ladderStateCode(s), methodCode(m), corr, age, st, eh, ev);
    return nmeaFinish(body, out, outSize);
}
