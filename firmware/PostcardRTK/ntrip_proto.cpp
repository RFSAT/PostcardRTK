// ntrip_proto.cpp - see ntrip_proto.h
#include "ntrip_proto.h"
#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <cstring>

size_t base64Encode(const uint8_t *in, size_t len, char *out, size_t outSize)
{
    static const char t[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    size_t need = ((len + 2) / 3) * 4 + 1;
    if (outSize < need)
        return 0;
    size_t o = 0;
    for (size_t i = 0; i < len; i += 3)
    {
        uint32_t v = (uint32_t)in[i] << 16;
        if (i + 1 < len)
            v |= (uint32_t)in[i + 1] << 8;
        if (i + 2 < len)
            v |= in[i + 2];
        out[o++] = t[(v >> 18) & 63];
        out[o++] = t[(v >> 12) & 63];
        out[o++] = (i + 1 < len) ? t[(v >> 6) & 63] : '=';
        out[o++] = (i + 2 < len) ? t[v & 63] : '=';
    }
    out[o] = 0;
    return o;
}

size_t ntripBuildRequest(char *out, size_t outSize, const char *host, uint16_t port, const char *mount,
                         const char *user, const char *pass, bool v2, const char *agent)
{
    char auth[240] = {0};
    if (user && user[0])
    {
        char cred[140];
        snprintf(cred, sizeof(cred), "%s:%s", user, pass ? pass : "");
        char b64[200];
        base64Encode((const uint8_t *)cred, strlen(cred), b64, sizeof(b64));
        snprintf(auth, sizeof(auth), "Authorization: Basic %s\r\n", b64);
    }
    int n;
    if (v2)
        n = snprintf(out, outSize,
                     "GET /%s HTTP/1.1\r\nHost: %s:%u\r\nNtrip-Version: Ntrip/2.0\r\nUser-Agent: NTRIP %s\r\n%s"
                     "Connection: close\r\n\r\n",
                     mount, host, port, agent, auth);
    else
        n = snprintf(out, outSize, "GET /%s HTTP/1.0\r\nUser-Agent: NTRIP %s\r\nAccept: */*\r\n%sConnection: close\r\n\r\n",
                     mount, agent, auth);
    return (n > 0 && (size_t)n < outSize) ? (size_t)n : 0;
}

void NtripResponse::reset()
{
    _len = 0;
    _status[0] = 0;
    _chunked = false;
    _result = NtripReply::Incomplete;
}

static bool containsNoCase(const char *hay, const char *needle)
{
    size_t n = strlen(needle);
    for (; *hay; hay++)
    {
        size_t i = 0;
        while (i < n && hay[i] && tolower((unsigned char)hay[i]) == tolower((unsigned char)needle[i]))
            i++;
        if (i == n)
            return true;
    }
    return false;
}

NtripReply NtripResponse::feed(const uint8_t *data, size_t len, size_t *bodyOffset)
{
    *bodyOffset = len;
    if (_result != NtripReply::Incomplete)
    {
        *bodyOffset = 0;
        return _result;
    }
    for (size_t i = 0; i < len; i++)
    {
        if (_len >= sizeof(_hdr) - 1)
            return _result = NtripReply::SoftError; // header too long
        _hdr[_len++] = (char)data[i];
        _hdr[_len] = 0;

        // Status line complete?
        if (_status[0] == 0)
        {
            char *eol = strstr(_hdr, "\r\n");
            if (!eol)
            {
                // Some casters terminate the status with a bare LF
                eol = strchr(_hdr, '\n');
                if (!eol)
                    continue;
            }
            size_t sl = (size_t)(eol - _hdr);
            if (sl >= sizeof(_status))
                sl = sizeof(_status) - 1;
            memcpy(_status, _hdr, sl);
            _status[sl] = 0;

            if (strncmp(_status, "ICY 200", 7) == 0)
            {
                // NTRIP 1.0: the stream starts right after the status line (extra CRLF is skipped by the framer)
                *bodyOffset = i + 1;
                return _result = NtripReply::Ok;
            }
            if (strncmp(_status, "SOURCETABLE", 11) == 0)
                return _result = NtripReply::Sourcetable;
            if (strncmp(_status, "HTTP/", 5) != 0)
                return _result = NtripReply::SoftError;
            if (containsNoCase(_status, "banned") || containsNoCase(_status, "sandbox"))
                return _result = NtripReply::SoftError;
            continue;
        }

        // HTTP: wait for the blank line ending the headers
        if (_len >= 4 && memcmp(&_hdr[_len - 4], "\r\n\r\n", 4) == 0)
        {
            const char *sp = strchr(_status, ' ');
            int code = sp ? (int)strtol(sp + 1, nullptr, 10) : 0;
            _chunked = containsNoCase(_hdr, "transfer-encoding: chunked");
            if (code == 200)
            {
                if (containsNoCase(_hdr, "gnss/sourcetable"))
                    return _result = NtripReply::Sourcetable;
                *bodyOffset = i + 1;
                return _result = NtripReply::Ok;
            }
            if (code == 401 || code == 403)
                return _result = NtripReply::Unauthorized;
            if (code == 404)
                return _result = NtripReply::Sourcetable;
            if (code >= 500 || code == 429 || code == 0)
                return _result = NtripReply::SoftError;
            return _result = NtripReply::HardError;
        }
    }
    *bodyOffset = len;
    return NtripReply::Incomplete;
}

static int hexDigit(uint8_t c)
{
    if (c >= '0' && c <= '9')
        return c - '0';
    c = (uint8_t)tolower(c);
    if (c >= 'a' && c <= 'f')
        return c - 'a' + 10;
    return -1;
}

size_t ChunkDecoder::decode(const uint8_t *in, size_t len, uint8_t *out)
{
    size_t o = 0;
    for (size_t i = 0; i < len; i++)
    {
        uint8_t c = in[i];
        switch (_state)
        {
        case SIZE: {
            int h = hexDigit(c);
            if (h >= 0)
            {
                _size = (_size << 4) | (uint32_t)h;
                if (_size > 0x100000)
                    _error = true;
            }
            else if (c == ';' || c == ' ')
                _state = EXT;
            else if (c == '\r')
                _state = SIZE_LF;
            else if (c == '\n')
                _state = (_size == 0) ? DONE : DATA;
            else
                _error = true;
            break;
        }
        case EXT:
            if (c == '\n')
                _state = (_size == 0) ? DONE : DATA;
            break;
        case SIZE_LF:
            _state = (_size == 0) ? DONE : DATA;
            break;
        case DATA:
            out[o++] = c;
            if (--_size == 0)
                _state = DATA_CR;
            break;
        case DATA_CR:
            _state = (c == '\r') ? DATA_LF : SIZE;
            if (c != '\r' && c != '\n')
                _error = true;
            if (c == '\n')
                _state = SIZE;
            break;
        case DATA_LF:
            _state = SIZE;
            break;
        case DONE:
            break;
        }
    }
    return o;
}
