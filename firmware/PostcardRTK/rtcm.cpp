// rtcm.cpp - see rtcm.h
#include "rtcm.h"

uint32_t rtcmCrc24q(const uint8_t *data, size_t len)
{
    uint32_t crc = 0;
    for (size_t i = 0; i < len; i++)
    {
        crc ^= (uint32_t)data[i] << 16;
        for (int b = 0; b < 8; b++)
        {
            crc <<= 1;
            if (crc & 0x1000000)
                crc ^= 0x1864CFB;
        }
    }
    return crc & 0xFFFFFF;
}

bool RtcmFramer::feed(uint8_t b)
{
    if (_len == 0)
    {
        if (b != 0xD3)
            return false; // hunting for the preamble
        _buf[_len++] = b;
        return false;
    }
    _buf[_len++] = b;
    if (_len == 3)
    {
        // 6 reserved bits must be zero, then a 10-bit length
        if (_buf[1] & 0xFC)
        {
            _len = 0;
            if (b == 0xD3)
                _buf[_len++] = b;
            return false;
        }
        size_t payload = ((size_t)(_buf[1] & 0x03) << 8) | _buf[2];
        _need = 3 + payload + 3;
        return false;
    }
    if (_len < 3 || _len < _need)
        return false;

    // Complete frame
    size_t n = _len;
    _len = 0;
    uint32_t crc = rtcmCrc24q(_buf, n - 3);
    uint32_t got = ((uint32_t)_buf[n - 3] << 16) | ((uint32_t)_buf[n - 2] << 8) | _buf[n - 1];
    if (crc != got)
    {
        _crcErrors++;
        return false;
    }
    _frameLen = n;
    _msgType = (n >= 8) ? (uint16_t)((_buf[3] << 4) | (_buf[4] >> 4)) : 0;
    if ((_msgType == 1005 || _msgType == 1006) && n >= 9)
        _station = ((_buf[4] & 0x0F) << 8) | _buf[5];
    _frames++;
    return true;
}
