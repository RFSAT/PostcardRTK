// selector.cpp - see selector.h
#include "selector.h"

SourceSelector::SourceSelector()
{
    static const uint32_t def[] = {15, 30, 60, 120, 300};
    setBackoff(def, 5);
}

void SourceSelector::setBackoff(const uint32_t *sec, int n)
{
    if (n > SEL_MAX_BACKOFF)
        n = SEL_MAX_BACKOFF;
    for (int i = 0; i < n; i++)
        _backoffSec[i] = sec[i];
    _backoffCount = n;
}

void SourceSelector::setSources(int count, const bool *v)
{
    if (count > SEL_MAX_SOURCES)
        count = SEL_MAX_SOURCES;
    _count = count;
    for (int i = 0; i < SEL_MAX_SOURCES; i++)
        _valid[i] = (i < count) ? v[i] : false;
    reset(true);
}

bool SourceSelector::valid(int i) const
{
    if (i < 0 || i >= _count || !_valid[i])
        return false;
    // Without failover only the first configured source is used
    if (!_failover)
    {
        for (int j = 0; j < i; j++)
            if (_valid[j])
                return false;
    }
    return true;
}

bool SourceSelector::usable(int i) const { return valid(i) && !disabled(i); }

int SourceSelector::usableCount() const
{
    int n = 0;
    for (int i = 0; i < _count; i++)
        n += usable(i) ? 1 : 0;
    return n;
}

int SourceSelector::validCount() const
{
    int n = 0;
    for (int i = 0; i < _count; i++)
        n += valid(i) ? 1 : 0;
    return n;
}

int SourceSelector::firstUsable() const
{
    for (int i = 0; i < _count; i++)
        if (usable(i))
            return i;
    return 0;
}

void SourceSelector::reset(bool clearDisabled)
{
    if (clearDisabled)
    {
        _disabledMask = 0;
        _backoffLevel = 0;
    }
    _cycleFailures = 0;
    _isConnected = false;
    _active = firstUsable();
}

void SourceSelector::connected(uint32_t nowMs)
{
    _cycleFailures = 0;
    _backoffLevel = 0;
    _connectedAt = nowMs;
    _isConnected = true;
}

bool SourceSelector::failbackDue(uint32_t nowMs) const
{
    if (!_failover || _failbackSec == 0 || !_isConnected)
        return false;
    if (_active <= firstUsable())
        return false;
    return (nowMs - _connectedAt) >= _failbackSec * 1000UL;
}

SelectorDecision SourceSelector::fail(bool hard, uint32_t nowMs)
{
    (void)nowMs;
    SelectorDecision d;
    int failed = _active;
    _isConnected = false;
    if (hard && failed >= 0 && failed < SEL_MAX_SOURCES)
        _disabledMask |= (1u << failed);

    int usableN = usableCount();
    if (usableN == 0)
    {
        d.stopped = true;
        d.source = failed;
        return d;
    }

    // Next usable source after the failed one (may be the same one)
    int next = failed;
    for (int n = 1; n <= _count; n++)
    {
        int i = (failed + n) % _count;
        if (usable(i))
        {
            next = i;
            break;
        }
    }

    _cycleFailures++;
    d.delayMs = _retryMs;
    if (_cycleFailures >= usableN)
    {
        // Every usable source failed in a row: back off, then restart from the top
        _cycleFailures = 0;
        int lvl = (_backoffLevel < _backoffCount) ? _backoffLevel : _backoffCount - 1;
        d.delayMs = (_backoffCount > 0 ? _backoffSec[lvl] : 60) * 1000UL;
        if (_backoffLevel < 1000)
            _backoffLevel++;
        next = firstUsable();
        d.backoff = true;
    }
    d.switched = (next != failed);
    if (d.switched)
        _switches++;
    _active = next;
    d.source = next;
    return d;
}
