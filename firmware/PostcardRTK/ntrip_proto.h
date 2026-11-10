// ntrip_proto.h - NTRIP request building, response parsing and HTTP chunk decoding
// (portable, unit-tested on the PC; the socket handling lives in ntrip.cpp)
#pragma once
#include <cstddef>
#include <cstdint>

size_t base64Encode(const uint8_t *in, size_t len, char *out, size_t outSize);

// Build the GET request for a mount point. v2 = NTRIP 2.0 (HTTP/1.1), else NTRIP 1.0.
size_t ntripBuildRequest(char *out, size_t outSize, const char *host, uint16_t port, const char *mount,
                         const char *user, const char *pass, bool v2, const char *agent);

enum class NtripReply
{
    Incomplete,   // need more bytes
    Ok,           // stream follows
    Sourcetable,  // mount point not found (caster sent its table)
    Unauthorized, // 401
    SoftError,    // try again later (5xx, banned, sandbox, garbage)
    HardError,    // other 4xx
};

class NtripResponse
{
  public:
    void reset();
    // Feed received bytes. Consumes header bytes; once Ok, *bodyOffset tells where the stream starts.
    NtripReply feed(const uint8_t *data, size_t len, size_t *bodyOffset);
    bool chunked() const { return _chunked; }
    const char *statusLine() const { return _status; }

  private:
    char _hdr[768];
    size_t _len = 0;
    char _status[96] = {0};
    bool _chunked = false;
    NtripReply _result = NtripReply::Incomplete;
};

// Decodes HTTP/1.1 "Transfer-Encoding: chunked" in place
class ChunkDecoder
{
  public:
    void reset() { _state = SIZE; _size = 0; _error = false; }
    // Decode len bytes of in into out (out may equal in); returns the payload length
    size_t decode(const uint8_t *in, size_t len, uint8_t *out);
    bool error() const { return _error; }

  private:
    enum { SIZE, EXT, SIZE_LF, DATA, DATA_CR, DATA_LF, DONE } _state = SIZE;
    uint32_t _size = 0;
    bool _error = false;
};
