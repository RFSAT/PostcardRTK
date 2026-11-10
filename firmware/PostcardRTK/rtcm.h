// rtcm.h - RTCM 3 framing and CRC-24Q check (portable, unit-tested on the PC)
#pragma once
#include <cstddef>
#include <cstdint>

static const size_t RTCM_MAX_FRAME = 1023 + 6;

uint32_t rtcmCrc24q(const uint8_t *data, size_t len);

// Finds RTCM 3 frames in a byte stream (an NTRIP stream may carry other bytes,
// for example caster text). Only frames with a valid CRC are returned, so the
// receiver never sees damaged messages.
class RtcmFramer
{
  public:
    // Feed one byte; returns true when a complete valid frame is in frame()
    bool feed(uint8_t b);
    const uint8_t *frame() const { return _buf; }
    size_t frameLength() const { return _frameLen; }
    uint16_t messageType() const { return _msgType; }

    uint32_t frames() const { return _frames; }
    uint32_t crcErrors() const { return _crcErrors; }
    int stationId() const { return _station; } // from 1005/1006, -1 until seen

  private:
    uint8_t _buf[RTCM_MAX_FRAME];
    size_t _len = 0;
    size_t _need = 0;
    size_t _frameLen = 0;
    uint16_t _msgType = 0;
    uint32_t _frames = 0, _crcErrors = 0;
    int _station = -1;
};
