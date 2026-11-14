// ublox.cpp - see ublox.h
#include "ublox.h"
#include <cstdio>
#include <cstring>

size_t ubxFrame(uint8_t cls, uint8_t id, const uint8_t *payload, uint16_t len, uint8_t *out, size_t outSize)
{
    const size_t n = (size_t)len + 8;
    if (n > outSize)
        return 0;
    out[0] = 0xB5;
    out[1] = 0x62;
    out[2] = cls;
    out[3] = id;
    out[4] = len & 0xFF;
    out[5] = len >> 8;
    if (len)
        memcpy(out + 6, payload, len);
    uint8_t a = 0, b = 0;
    for (size_t i = 2; i < 6 + (size_t)len; i++)
    {
        a += out[i];
        b += a;
    }
    out[6 + len] = a;
    out[7 + len] = b;
    return n;
}

static int keyBytes(uint32_t id)
{
    switch ((id >> 28) & 7)
    {
    case 1:
    case 2: return 1;
    case 3: return 2;
    case 4: return 4;
    case 5: return 8;
    default: return 0;
    }
}

size_t ubxValset(const UbxKey &k, uint8_t *out, size_t outSize)
{
    uint8_t p[4 + 4 + 8] = {0x00, 0x03, 0x00, 0x00}; // version 0, layers RAM + BBR, reserved
    const int vb = keyBytes(k.id);
    if (!vb)
        return 0;
    for (int i = 0; i < 4; i++)
        p[4 + i] = (k.id >> (8 * i)) & 0xFF;
    for (int i = 0; i < vb; i++)
        p[8 + i] = i < 4 ? (k.value >> (8 * i)) & 0xFF : 0;
    return ubxFrame(0x06, 0x8A, p, (uint16_t)(8 + vb), out, outSize);
}

size_t ubxMonVerPoll(uint8_t *out, size_t outSize) { return ubxFrame(0x0A, 0x04, nullptr, 0, out, outSize); }

// Applied at every start-up. All are standard ZED-F9 keys; one VALSET each, so a key a firmware version lacks only
// NAKs itself (shown in the event log) and the others still apply.
const UbxKey UBX_SETUP[] = {
    {0x10730001, 1, "UART1 in UBX"},
    {0x10730002, 1, "UART1 in NMEA"},
    {0x10730004, 1, "UART1 in RTCM3"},            // corrections from NTRIP or a phone app
    {0x10740001, 1, "UART1 out UBX"},             // ACK / NAK and MON-VER replies
    {0x10740002, 1, "UART1 out NMEA"},
    {0x10930006, 1, "NMEA high precision"},       // 7 decimals of a minute (~0.2 mm) instead of 5 (~2 cm)
    {0x209100bb, 1, "GGA on UART1"},
    {0x209100ac, 1, "RMC on UART1"},
    {0x209100c0, 1, "GSA on UART1"},
    {0x209100c5, 1, "GSV on UART1"},
    {0x209100d4, 1, "GST on UART1"},              // accuracy estimate for apps and the method ladder
    {0x209100b1, 1, "VTG on UART1"},
    {0x209100ca, 0, "GLL off on UART1"},          // duplicate of GGA, saves bandwidth
    // The same for UART2: which ZED-F9R UART the ESP_F9x board wires to the ESP32 is not documented
    {0x10750001, 1, "UART2 in UBX"},
    {0x10750002, 1, "UART2 in NMEA"},
    {0x10750004, 1, "UART2 in RTCM3"},
    {0x10760001, 1, "UART2 out UBX"},
    {0x10760002, 1, "UART2 out NMEA"},
    {0x209100bc, 1, "GGA on UART2"},
    {0x209100ad, 1, "RMC on UART2"},
    {0x209100c1, 1, "GSA on UART2"},
    {0x209100c6, 1, "GSV on UART2"},
    {0x209100d5, 1, "GST on UART2"},
    {0x209100b2, 1, "VTG on UART2"},
    {0x209100cb, 0, "GLL off on UART2"},
    {0x10310020, 1, "SBAS signals"},              // EGNOS
    {0x10360004, 1, "SBAS differential corrections"},
    {0x10060027, 1, "IMU mount auto-alignment"},  // dead reckoning without a manual mounting angle
};
const int UBX_SETUP_COUNT = sizeof(UBX_SETUP) / sizeof(UBX_SETUP[0]);

void UbxParser::finish(UbxEvent &ev)
{
    _frames++;
    if (_cls == 0x05 && _len >= 2)
    {
        _ackCls = _buf[0];
        _ackId = _buf[1];
        ev = _id == 0x01 ? UBX_ACK : UBX_NAK;
        return;
    }
    if (_cls == 0x0A && _id == 0x04 && _len >= 40)
    {
        // swVersion[30] hwVersion[10] then extensions of 30 characters: keep FWVER=, PROTVER= and MOD=
        char sw[31];
        memcpy(sw, _buf, 30);
        sw[30] = 0;
        char fw[31] = "", prot[31] = "", mod[31] = "";
        const uint16_t kept = _len < sizeof(_buf) ? _len : sizeof(_buf);
        for (uint16_t off = 40; off + 30 <= kept; off += 30)
        {
            char e[31];
            memcpy(e, _buf + off, 30);
            e[30] = 0;
            if (!strncmp(e, "FWVER=", 6))
                snprintf(fw, sizeof(fw), "%s", e + 6);
            else if (!strncmp(e, "PROTVER=", 8))
                snprintf(prot, sizeof(prot), "%s", e + 8);
            else if (!strncmp(e, "MOD=", 4))
                snprintf(mod, sizeof(mod), "%s", e + 4);
        }
        snprintf(_ver, sizeof(_ver), "%s%s%s%s%s", fw[0] ? fw : sw, prot[0] ? ", protocol " : "", prot, mod[0] ? ", " : "", mod);
        ev = UBX_MONVER;
        return;
    }
    ev = UBX_OTHER;
}

UbxEvent UbxParser::feed(uint8_t b)
{
    UbxEvent ev = UBX_NONE;
    switch (_state)
    {
    case 0:
        if (b == 0xB5)
            _state = 1;
        break;
    case 1:
        _state = b == 0x62 ? 2 : (b == 0xB5 ? 1 : 0);
        break;
    case 2:
        _cls = b;
        _ckA = b;
        _ckB = _ckA;
        _state = 3;
        break;
    case 3:
        _id = b;
        _ckA += b;
        _ckB += _ckA;
        _state = 4;
        break;
    case 4:
        _len = b;
        _ckA += b;
        _ckB += _ckA;
        _state = 5;
        break;
    case 5:
        _len |= (uint16_t)b << 8;
        _ckA += b;
        _ckB += _ckA;
        _pos = 0;
        if (_len > 1024) // not a frame we expect: resynchronise
        {
            _errors++;
            _state = 0;
        }
        else
            _state = _len ? 6 : 7;
        break;
    case 6:
        if (_pos < sizeof(_buf))
            _buf[_pos] = b;
        _pos++;
        _ckA += b;
        _ckB += _ckA;
        if (_pos >= _len)
            _state = 7;
        break;
    case 7:
        _rxA = b;
        _state = 8;
        break;
    case 8:
        if (_rxA == _ckA && b == _ckB)
            finish(ev);
        else
            _errors++;
        _state = 0;
        break;
    }
    return ev;
}
