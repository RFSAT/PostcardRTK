// evtrim.cpp - see evtrim.h
#include "evtrim.h"
#include <cstdlib>
#include <vector>

uint32_t evSession(const char *s, size_t n)
{
    if (n < 3 || s[1] != '\t')
        return 0;
    return (uint32_t)strtoul(s + 2, nullptr, 10);
}

std::string evTrim(const std::string &log, uint32_t cur, size_t maxBytes)
{
    if (log.size() <= maxBytes)
        return log;
    struct Ln
    {
        size_t off, len;
        uint32_t sess;
        char kind;
        bool keep;
    };
    std::vector<Ln> lines;
    for (size_t i = 0; i < log.size();)
    {
        size_t j = log.find('\n', i);
        size_t len = (j == std::string::npos ? log.size() : j + 1) - i;
        lines.push_back({i, len, evSession(log.data() + i, len), log[i], true});
        i += len;
    }
    size_t total = log.size();
    // 1. earlier sessions, oldest lines first
    for (auto &l : lines)
    {
        if (total <= maxBytes)
            break;
        if (l.sess != cur)
        {
            l.keep = false;
            total -= l.len;
        }
    }
    // 2. still too big: the current session's oldest events (its start line stays)
    size_t dropped = 0, prevDropped = 0;
    const size_t markerLen = 32;
    if (total > maxBytes)
    {
        for (auto &l : lines) // an earlier marker is replaced by one with the new total
            if (l.keep && l.kind == 'D' && l.sess == cur)
            {
                prevDropped += strtoul(log.c_str() + l.off + 2 + std::to_string(cur).size() + 1, nullptr, 10);
                l.keep = false;
                total -= l.len;
            }
        for (auto &l : lines)
        {
            if (total + markerLen <= maxBytes)
                break;
            if (l.keep && l.sess == cur && l.kind == 'E')
            {
                l.keep = false;
                total -= l.len;
                dropped++;
            }
        }
    }
    std::string out;
    out.reserve(total + markerLen);
    bool markerDone = !(dropped + prevDropped);
    for (auto &l : lines)
    {
        if (!l.keep)
            continue;
        out.append(log, l.off, l.len);
        if (!markerDone && l.kind == 'S' && l.sess == cur)
        {
            out += "D\t" + std::to_string(cur) + "\t" + std::to_string(dropped + prevDropped) + "\n";
            markerDone = true;
        }
    }
    if (!markerDone) // the start line itself was not in the file: put the marker first
        out = "D\t" + std::to_string(cur) + "\t" + std::to_string(dropped + prevDropped) + "\n" + out;
    return out;
}
